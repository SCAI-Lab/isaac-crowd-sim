import subprocess
from typing import List

from bs4 import BeautifulSoup
from bs4.element import NavigableString, Tag


def main(folder_path: str, file_name: str, output_file: str) -> None:
    with open(f"{folder_path}/{file_name}.sdf", "r") as f:
        data = f.read()

    bs_data = BeautifulSoup(data, "xml")

    models: List[str] = []
    included_models = bs_data.find_all("include")

    with open(output_file, "w") as output:
        for model in included_models:
            uri: Tag = model.find_all("uri")[0]
            element: NavigableString = uri.contents[0]
            element_name = str(element).split("/")[2]  # general formulation model://name
            element_name = element_name.replace(" ", "_")
            if "airport" in element_name or "sun" in element_name:
                continue

            if element_name not in models:
                models.append(element_name)

            pose: Tag = model.find_all("pose")[0]
            data = str(pose.contents[0])
            row = f"{element_name},{data}\n".replace(" ", ",")
            output.write(row)

    for model in models:
        if "_" in model or model == "Barrier":
            print(
                f"The following model has been skipped: {model}. Please do the conversion manually"
            )
            continue
        subfolder = f"{folder_path}/models/{model}"
        subprocess.Popen([f"./generate_usd.sh {subfolder}"], shell=True)

    print("All usd models generated!")


if __name__ == "__main__":
    folder_path = "../../assets/airport/output"
    file_name = "airport"
    output_file_path = "../../config/airport_models.csv"
    main(folder_path, file_name, output_file_path)
