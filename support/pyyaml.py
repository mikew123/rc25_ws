# test python yaml

import yaml
from pathlib import Path
from pprint import pprint

file = "~/sambashare/nav_files/navto_cones.yml"

def load_and_print_yaml(path_str: str) -> None:
    path = Path(path_str).expanduser()
    if not path.exists():
        print(f"YAML file not found: {path}", file=sys.stderr)
        return
    try:
        with path.open("r", encoding="utf-8") as f:
            docs = list(yaml.safe_load_all(f))
    except Exception as e:
        print(f"Failed to read/parse YAML: {e}", file=sys.stderr)
        return

    if not docs:
        print("YAML file is empty")
        return

    if len(docs) == 1:
        pprint(docs[0])
    else:
        for i, doc in enumerate(docs):
            print(f"--- document {i} ---")
            pprint(doc)

if __name__ == "__main__":
    load_and_print_yaml(file)



