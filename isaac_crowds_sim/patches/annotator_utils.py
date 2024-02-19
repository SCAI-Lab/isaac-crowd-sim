# Move this file to the location extscache/omni.replicator.core-1.10.10+105.1.lx64.r.cp310/omni/replicator/core/scripts/utils/annotator_utils.py

import asyncio
import ctypes
import json
import re
import sys
from typing import Dict, Tuple

import carb
import numpy as np
import omni.graph.core as og
import warp as wp
from omni.graph.scriptnode._impl.extension import (
    SCRIPTNODE_OPT_IN_SETTING,
    is_check_enabled,
    verify_scriptnode_load,
)

SEMANTIC_SPLIT_REGEX = "\s(?=\w+:)"
NUMPY_TO_WARP_MAP = {
    np.int8: wp.int8,
    np.byte: wp.int8,
    np.int16: wp.int16,
    np.short: wp.int16,
    np.int32: wp.int32,
    np.intc: wp.int32,
    np.int64: wp.int64,
    np.uint8: wp.uint8,
    np.ubyte: wp.uint8,
    np.uint16: wp.uint16,
    np.ushort: wp.uint16,
    np.uint32: wp.int32,  # PyTorch does not support uint32
    np.uintc: wp.int32,  # PyTorch does not support uint32
    np.uint64: wp.uint64,
    np.float16: wp.float16,
    np.float32: wp.float32,
    np.float64: wp.float64,
}
BASEDATATYPE_TO_NUMPY_MAP = {
    og.BaseDataType.DOUBLE: np.float64,
    og.BaseDataType.FLOAT: np.float32,
    og.BaseDataType.HALF: np.float16,
    og.BaseDataType.INT: np.int32,
    og.BaseDataType.INT64: np.int64,
    og.BaseDataType.UCHAR: np.uint8,
    og.BaseDataType.UINT: np.uint32,
    og.BaseDataType.UINT64: np.uint64,
}
NORMAL_ATTRIBUTE_NAMES = [
    "exec",
    "height",
    "width",
    "bufferSize",
    "data",
    "dataPtr",
    "cudaStream",
    "format",
    "__device",
    "__do_array_copy",
    "cudaDeviceIndex",
    "strides",
    "Ptr",
    "dataType",
    "dataShape",
]
ARRAY_PARAMS = ["height", "width", "bufferSize"]
PTR_ARRAY_PARAMS = [
    "bufferSize",
    "ptr",
    "dataType",
    "dataShape",
    "strides",
    "cudaDeviceIndex",
    "height",
    "width",
]
DATA_PARAMS = ["dataPtr", "data"]


def _get_dtype(elem_type):
    if isinstance(elem_type, str):
        # Convert to numpy type
        try:
            elem_type = np.dtype(elem_type).type
        except:
            raise ValueError(f"Data type {elem_type} not supported for wrapping with warp")

    if elem_type in NUMPY_TO_WARP_MAP:
        return NUMPY_TO_WARP_MAP[elem_type]
    elif isinstance(elem_type, np.dtype):
        # Fallback on uint8
        return wp.uint8
    else:
        raise ValueError(f"Data type {elem_type} not supported for wrapping with warp")


def _get_strides(strides_raw, warp_data_type):
    if np.all(strides_raw != 0):
        return (
            strides_raw[1],
            strides_raw[0],
            wp.types.type_size_in_bytes(warp_data_type),
        )
    else:
        return None


def _get_address(attr):
    ptr_type = ctypes.POINTER(ctypes.c_size_t)
    ptr = ctypes.cast(attr.memory, ptr_type)
    return ptr.contents.value


def _get_pointer(param):
    data_view = og.AttributeValueHelper(param)
    data_view.gpu_ptr_kind = og.PtrToPtrKind.CPU
    data = data_view.get(on_gpu=True)
    return _get_address(data)


def _reshape_output_attr(
    attr,
    height,
    width,
    strides,
    buffer_size,
    elem_type,
    elem_count,
    is2DArray,
    source_device,
    target_device,
    dtype,
    output_array=None,
    **kwargs,
):
    if source_device.startswith("cuda"):
        ptr = _get_pointer(attr)
        return _reshape_output_ptr(
            ptr,
            height,
            width,
            strides,
            buffer_size,
            elem_type,
            elem_count,
            source_device,
            target_device,
            dtype,
            output_array,
        )
    else:
        data = attr.get_array(False, False, 0).copy()

        if (data is None) or (len(data) < np.dtype(elem_type).itemsize):
            if is2DArray:
                shape = (0, 0, elem_count) if elem_count > 1 else (0, 0)
            else:
                shape = (0, elem_count) if elem_count > 1 else (0)
            return np.empty(shape, elem_type)

        # If datatype is default uchar, recast
        if data.dtype == np.uint8 and elem_type != np.uint8:
            data = data.view(elem_type)
        assert len(data) > 0
        if not is2DArray:
            return (
                data.reshape(data.shape[0] // elem_count, elem_count) if elem_count > 1 else data
            )

        data = np.squeeze(data.reshape(height, width, -1))
        if target_device == "cpu":
            return data
        else:
            return wp.array(data, device=target_device)


@carb.profiler.profile
def _reshape_output_ptr(
    ptr,
    height,
    width,
    strides,
    buffer_size,
    elem_type,
    elem_count,
    source_device,
    target_device,
    dtype,
    **kwargs,
):
    if "shape" in kwargs:
        shape = tuple(kwargs.get("shape"))
    elif buffer_size and width and height and dtype:
        elem_count = buffer_size // height // width // wp.types.type_size_in_bytes(dtype)
        shape = (height, width, elem_count)
    elif height and width and elem_count:
        shape = (height, width, elem_count)
    else:
        shape = buffer_size // wp.types.type_size_in_bytes(dtype)
    carb.profiler.begin(1, "Wrap warp array")
    data = wp.types.array(
        dtype=dtype,
        shape=shape,
        strides=strides,
        ptr=ptr,
        device=source_device,
        owner=False,
        requires_grad=False,
    )
    # Calculate element count
    if buffer_size and width and height and elem_type:
        elem_count = buffer_size // width // height // elem_type(0).itemsize

    # Data needs to be squeezed after retrieval otherwise strides don't match data
    if height and width and elem_count == 1:
        if not data.is_contiguous:
            data = data.contiguous()
        data = data.reshape((height, width))
    carb.profiler.end(1)

    if not target_device.startswith("cuda"):
        if data.size > 0:  # TEMP fix because warp errors out when having empty array.
            data = np.squeeze(data.numpy())
            data = data.view(elem_type)
            return data
        else:
            data = np.array([])
    return data


def _resolve_data_type(data_type_attr):
    try:
        data_params = {}
        resolved_type = data_type_attr.get_resolved_type()
        if resolved_type.base_type == og.BaseDataType.TOKEN:
            if data_type_attr.get():
                np_type = np.dtype("".join(data_type_attr.get())).type
            else:
                return {}
        else:
            np_type = BASEDATATYPE_TO_NUMPY_MAP[resolved_type.base_type]
            data_params["elem_count"] = resolved_type.tuple_count
        data_params["elem_type"] = np_type
        data_params["dtype"] = _get_dtype(data_params["elem_type"])
        return data_params
    except ValueError as e:
        raise ValueError(f"Error processing node attribute `{data_type_attr.get_name()}`: {e}")


def _update_params_for_data(data_params, node_params, data_name):
    if f"{data_name}Height" in node_params:
        data_params["height"] = node_params[f"{data_name}Height"].get()
    if f"{data_name}Width" in node_params:
        data_params["width"] = node_params[f"{data_name}Width"].get()
    if f"{data_name}BufferSize" in node_params:
        data_params["buffer_size"] = node_params[f"{data_name}BufferSize"].get()
    if f"{data_name}CudaDeviceIndex" in node_params:
        device_idx = node_params[f"{data_name}CudaDeviceIndex"].get()
        data_params["source_device"] = "cpu" if device_idx < 0 else f"cuda:{device_idx}"
    if f"{data_name}DataType" in node_params:
        data_params.update(_resolve_data_type(node_params[f"{data_name}DataType"]))
        if f"{data_name}Width" not in node_params:
            data_params["width"] = (
                data_params["buffer_size"]
                // data_params["elem_type"](0).itemsize
                // data_params["elem_count"]
            )
    if f"{data_name}Strides" in node_params:
        data_params["strides"] = _get_strides(
            node_params[f"{data_name}Strides"].get(), NUMPY_TO_WARP_MAP[data_params["elem_type"]]
        )
    if f"{data_name}DataShape" in node_params:
        data_params["shape"] = node_params[f"{data_name}DataShape"].get()
    return data_params


class AnnotatorCache:
    """Cache static annotator parameters to improve performance

    Accessing OmniGraph attributes is relatively expensive. Cache static attributes where possible to improve
    performance in certain scenarios.
    """

    annotators = {}

    @classmethod
    @carb.profiler.profile
    def get_data(cls, annotator_id, node_params, annotator_params, target_device, **kwargs):
        params = cls.get_common_params(annotator_id, node_params, annotator_params, target_device)
        device_idx = (
            node_params["cudaDeviceIndex"].get() if "cudaDeviceIndex" in node_params else -1
        )
        params["source_device"] = "cpu" if device_idx < 0 else f"cuda:{device_idx}"

        # Get alternate data names
        data_names = [n.split("Ptr")[0] for n in node_params if n.endswith("Ptr")]

        do_copy = kwargs.get("do_copy") if kwargs.get("do_copy") else params.get("do_array_copy")

        if data_names:
            data_outputs = {}
            for data_name in data_names:
                data_params = params.copy()
                data_params["ptr"] = node_params[f"{data_name}Ptr"].get()
                if data_name != "data":
                    # TODO jlafleche cache alternate data attributes
                    _update_params_for_data(data_params, node_params, data_name)

                # Get default parameters that shouldn't be cached
                if data_name == "data" and f"{data_name}Type" in node_params:
                    data_params.update(_resolve_data_type(node_params[f"{data_name}Type"]))

                # Get parameters that shouldn't be cached
                if f"{data_name}DataType" in node_params:
                    data_params.update(_resolve_data_type(node_params[f"{data_name}DataType"]))

                data = cls._get_data_from_ptr(annotator_id, data_params, do_copy)
                # if len(data) == 0:
                if data.size == 0:
                    # Empty array, delete cache
                    cls.clear(annotator_id)
                data_outputs[data_name] = data
            return data_outputs

        elif "data" in node_params and params["source_device"] != "cpu":
            params["ptr"] = _get_pointer(node_params["data"].get())
            return {"data": cls._get_data_from_ptr(annotator_id, params, do_copy)}

        elif "data" in node_params:
            params["attr"] = node_params["data"]
            data = _reshape_output_attr(**params)
            if len(data) == 0:
                # Empty array, delete cache
                cls.clear(annotator_id)
            return {"data": data}
        else:
            return None

    @classmethod
    @carb.profiler.profile
    def _get_data_from_ptr(cls, annotator_id, params, do_copy):
        array = _reshape_output_ptr(**params)

        if do_copy:
            if isinstance(array, np.ndarray) and params.get("source_device", "").lower() == "cpu":
                # If warp array is on CPU, `.numpy()` is a zero-copy operation
                # and requires a copy to avoid buffer being overwritten
                return array.copy()

            elif isinstance(array, wp.array):
                # Create a copy array to manage lifetime
                cached_arrays = cls.annotators[annotator_id].get("output_array")

                # Check if attributes have changed and create a new cache if they have
                if (
                    not cached_arrays
                    or cached_arrays[0].shape != array.shape
                    or cached_arrays[0].dtype != array.dtype
                    or cached_arrays[0].strides != array.strides
                ):
                    cls.annotators[annotator_id]["output_array"] = []
                    cached_arrays = cls.annotators[annotator_id].get("output_array")

                cached_array = None
                for idx in range(len(cached_arrays)):
                    if sys.getrefcount(cached_arrays[idx]) == 2:
                        cached_array = cached_arrays[idx]
                        break

                if cached_array is None:
                    carb.profiler.begin(1, f"{annotator_id}: warp array allocation")
                    cached_array = wp.empty_like(array, device=params["target_device"])
                    cls.annotators[annotator_id].setdefault("output_array", []).append(
                        cached_array
                    )
                    carb.profiler.end(1)

                carb.profiler.begin(1, f"{annotator_id}: warp array copy")
                wp.copy(cached_array, array)
                carb.profiler.end(1)
                return cached_array
            else:
                return array
        else:
            return array

    @classmethod
    @carb.profiler.profile
    def get_common_params(cls, annotator_id, node_params, annotator_params, target_device):
        if annotator_id is not None and annotator_id in cls.annotators:
            common_params = cls.annotators[annotator_id]["common_params"].copy()
            if target_device:
                common_params["target_device"] = target_device
            return common_params
        else:
            if "strides" in node_params:
                strides_raw = node_params["strides"].get()
                if np.all(strides_raw != 0):
                    strides = (
                        strides_raw[1],
                        strides_raw[0],
                        wp.types.type_size_in_bytes(NUMPY_TO_WARP_MAP[annotator_params.data_type]),
                    )
                else:
                    strides = None
            else:
                strides = None

            elem_count = annotator_params.num_elems
            elem_type = annotator_params.data_type
            is2DArray = annotator_params.is_2d_array
            buffer_size = node_params["bufferSize"].get()
            height = node_params["height"].get()
            width = node_params["width"].get()

            dtype = _get_dtype(elem_type)
            if is2DArray and not buffer_size:
                buffer_size = height * width * elem_count * np.dtype(elem_type).itemsize
            elif buffer_size > 0 and height > 1 and width > 1:
                elem_count = buffer_size // height // width // wp.types.type_size_in_bytes(dtype)

            common_params = {
                "height": height,
                "width": width,
                "strides": strides,
                "buffer_size": buffer_size,
                "elem_type": elem_type,
                "elem_count": elem_count,
                "is2DArray": is2DArray,
                "target_device": target_device if target_device else node_params["__device"].get(),
                "dtype": dtype,
                "do_array_copy": node_params["__do_array_copy"].get(),
            }

            # Add optional params
            if "dataShape" in node_params:
                shape = node_params["dataShape"].get()
                if len(shape) > 0:
                    common_params["shape"] = shape

            if annotator_id is None:
                return common_params
            else:
                cls.annotators.setdefault(annotator_id, {})["common_params"] = common_params
                return cls.annotators[annotator_id]["common_params"]

    @classmethod
    def clear(cls, annotator_id=None):
        """Clear cache

        Args:
            annotator_id: Optionally specify the annotator to clear from the cache. If `None`, clear entire cache.
                Defaults to `None`.
        """
        if annotator_id:
            if annotator_id in cls.annotators:
                cls.annotators.pop(annotator_id)
        else:
            cls.annotators = {}


def _get_address(attr):
    ptr_type = ctypes.POINTER(ctypes.c_size_t)
    ptr = ctypes.cast(attr.memory, ptr_type)
    return ptr.contents.value


def _get_pointer(param):
    data_view = og.AttributeValueHelper(param)
    data_view.gpu_ptr_kind = og.PtrToPtrKind.CPU
    data = data_view.get(on_gpu=True)
    return _get_address(data)


@carb.profiler.profile
def get_extra_data(params):
    data_names = [n.split("Ptr")[0] for n in params.keys() if n.endswith("Ptr")]
    data_exclusions = [
        f"{dn}{an[0].capitalize()}{an[1:]}" for dn in data_names for an in PTR_ARRAY_PARAMS
    ]
    extra_data = {}
    for key in params.keys():
        if key in NORMAL_ATTRIBUTE_NAMES:
            continue

        if key in data_exclusions:
            continue

        item = params[key]
        if (
            item.get_type_name() == "string"
            and item.get_array(False, False, 0) is not None
            and ("idToLabels" in key or "idToSemantics" in key)
        ):
            array = item.get_array(False, False, 0)
            if len(array) == 0:
                id_to_labels = {}
            else:
                id_to_labels = json.loads("".join(array))
            extra_data[key] = id_to_labels
        elif (
            item.get_type_name() == "token[]"
            and item.get_array(False, False, 0) is not None
            and key == "labels"
            and "ids" in params
        ):
            if not params.get("ids"):
                raise ValueError("Label array is paired with id array, but id array is missing.")
            # TODO: Too specific. Need to handle more general case.
            label_array = item.get_array(False, False, 0)

            semantic_array = None
            if params.get("semantics"):
                semantic_array = params["semantics"].get_array(False, False, 0)

            id_to_labels = {}
            id_to_semantics = {}

            if len(label_array) != 0:
                id_array = params["ids"].get_array(False, False, 0).reshape((len(label_array), -1))
                for i in range(len(label_array)):
                    if len(id_array[i]) == 1:
                        key = int(id_array[i][0])
                    else:
                        key = tuple(id_array[i])

                    if ":" not in label_array[i]:  # Prim path
                        id_to_labels[key] = label_array[i]
                    else:
                        type_to_labels = {}

                        for type_to_label in re.split(SEMANTIC_SPLIT_REGEX, label_array[i]):
                            semantic_type, semantic_label = type_to_label.split(":")
                            type_to_labels[semantic_type] = semantic_label
                        id_to_labels[key] = type_to_labels

                    if ":" not in label_array[i]:  # Prim path
                        id_to_labels[key] = label_array[i]
                    else:
                        type_to_labels = {}

                        for type_to_label in re.split(SEMANTIC_SPLIT_REGEX, label_array[i]):
                            semantic_type, semantic_label = type_to_label.split(":")
                            type_to_labels[semantic_type] = semantic_label
                        id_to_labels[key] = type_to_labels

                    if semantic_array:
                        type_to_semantic = {}

                        for type_to_label in re.split(SEMANTIC_SPLIT_REGEX, semantic_array[i]):
                            semantic_type, semantic_label = type_to_label.split(":")
                            type_to_semantic[semantic_type] = semantic_label

                        id_to_semantics[key] = type_to_semantic
                extra_data["idToLabels"] = id_to_labels
                if semantic_array:
                    extra_data["idToSemantics"] = id_to_semantics
            else:
                extra_data["idToLabels"] = {}

                if params.get("semantics"):
                    extra_data["idToSemantics"] = {}

        elif key == "ids" or key == "semantics":
            # Skip ids and semantics because it is handled with label array.
            continue
        else:
            data_type = item.get_resolved_type()
            if data_type.array_depth == 0:
                extra_data[key] = item.get()
            else:
                extra_data[key] = item.get_array(False, False, 0)
    return extra_data


@carb.profiler.profile
def _get_annotator_data(
    node_params: Dict,
    annotator_params: Tuple,
    device: str = None,
    annotator_id: Tuple[str] = None,
    do_copy: bool = None,
):
    # Handle additional keys
    extra_data = get_extra_data(node_params)
    # FIXME: Make height and width not a required params here, because not all annotators are 2D
    if all([a in node_params for a in ARRAY_PARAMS]) and any(
        [a in node_params for a in DATA_PARAMS]
    ):
        annotator_output = AnnotatorCache.get_data(
            annotator_id, node_params, annotator_params, do_copy=do_copy, target_device=device
        )

        if extra_data:
            annotator_output["info"] = extra_data
        elif len(annotator_output) == 1:
            annotator_output = annotator_output[next(iter(annotator_output))]
    else:
        annotator_output = extra_data

    return annotator_output


@carb.profiler.profile
def get_annotator_data(
    node: og.Node,
    annotator_params: tuple,
    from_inputs: bool = False,
    device: str = "cpu",
    annotator_id: Tuple[str] = None,
    do_copy: bool = None,
):
    carb.profiler.begin(1, "get annotator attributes")
    prefix = "inputs" if from_inputs else "outputs"
    node_params = {}

    # FIXME: Because of OM-47286, can't pass a bundle for now.
    for attribute in node.get_attributes():
        name = attribute.get_name()

        if prefix not in name:
            continue
        attr_str = name.split(":")
        if len(attr_str) != 2:
            continue
        _, param = attr_str
        node_params[param] = attribute
    carb.profiler.end(1)
    try:
        return _get_annotator_data(node_params, annotator_params, device, annotator_id, do_copy)
    except ValueError as e:
        carb.log_error(f"Encountered an error retrieving data from annotator {annotator_id}. {e}")


def script_node_check(nodes: list = []):
    """Prompt user before enabling a script node

    Call this script in any node capable of executing arbitrary scripts within the node's
    `initialize` call.

    Note: Only one prompt attempt is made per session
    """
    settings = carb.settings.get_settings()
    has_been_prompted = settings.get("/omni/replicator/scriptNodePrompted")
    if is_check_enabled() and not has_been_prompted:
        # Check is enabled - see user already opted-in
        scriptnode_opt_in = settings.get(SCRIPTNODE_OPT_IN_SETTING)

        if not scriptnode_opt_in:
            # The check is enabled but they opted out, or haven't been prompted yet
            try:
                import omni.kit.window.popup_dialog  # noqa
            except ImportError:
                # Don't prompt in headless mode
                return
            settings.set("/omni/replicator/scriptNodePrompted", True)

            async def prompt():
                verify_scriptnode_load(nodes)
                # Wait a few frames before re-enabling prompt
                await omni.kit.app.get_app().next_update_async()
                await omni.kit.app.get_app().next_update_async()
                await omni.kit.app.get_app().next_update_async()
                settings.set("/omni/replicator/scriptNodePrompted", False)

            asyncio.ensure_future(prompt())


def check_should_run_script():
    return bool(carb.settings.get_settings().get(SCRIPTNODE_OPT_IN_SETTING))
