# Move this file to the location extscache/omni.anim.people-0.1.11/omni/anim/people/scripts/global_character_position_manager.py

from __future__ import annotations

import utils.global_character_positions


class GlobalCharacterPositionManager:
    """Global class which stores current and predicted positions of all characters and moving objects."""

    __instance: GlobalCharacterPositionManager = None

    def __init__(self):
        if self.__instance is not None:
            raise RuntimeError("Only one instance of GlobalCharacterPositionManager is allowed")
        self._character_positions = {}
        self._character_future_positions = {}
        self._character_radius = {}
        self._character_orientation = {}
        GlobalCharacterPositionManager.__instance = self

    def destroy(self):
        GlobalCharacterPositionManager.__instance = None

    def __del__(self):
        self.destroy()

    @classmethod
    def get_instance(cls) -> GlobalCharacterPositionManager:
        if cls.__instance is None:
            GlobalCharacterPositionManager()
        return cls.__instance

    def set_character_radius(self, char_prim_path, radius):
        self._character_radius[char_prim_path] = radius
        utils.global_character_positions._character_radius[char_prim_path] = radius

    def get_character_radius(self, char_prim_path):
        return self._character_radius[char_prim_path]

    def set_character_current_pos(self, char_prim_path, pos):
        self._character_positions[char_prim_path] = pos
        utils.global_character_positions._character_positions[char_prim_path] = pos

    def set_character_future_pos(self, char_prim_path, pos):
        self._character_future_positions[char_prim_path] = pos
        utils.global_character_positions._character_future_positions[char_prim_path] = pos

    def set_character_orientation(self, char_prim_path, orientation):
        self._character_orientation[char_prim_path] = orientation

    def get_character_current_pos(self, char_prim_path):
        return self._character_positions[char_prim_path]

    def get_character_future_pos(self, char_prim_path):
        return self._character_future_positions[char_prim_path]

    def get_all_character_pos(self):
        return self._character_positions.values()

    def get_all_character_future_pos(self):
        return self._character_future_positions.values()

    def get_all_managed_characters(self):
        return self._character_positions.keys()
