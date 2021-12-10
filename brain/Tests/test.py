from pathlib import Path
from datetime import datetime

FILE = Path(__file__).parent.joinpath(datetime.now().strftime("%d/%m/%Y-%H:%M:%S") + ".log")
print(FILE)
try:
    FILE.touch()
except OSError:
    print("Erreur")
    exit(1)