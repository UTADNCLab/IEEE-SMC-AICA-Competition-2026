start "" quarc_run -q -Q -t tcpip://localhost:17000 *.rt-win64

start "" quarc_run -q -Q -t tcpip://localhost:17000 *.rt-win64

python setup_env.py

start "Virtual QDrone Model" "quarc_run" -D -r -t tcpip://localhost:17000  virtual_FlightStack.rt-win64 -uri tcpip://localhost:17002

TIMEOUT /T 1

start "Virtual QCar Model" "quarc_run" -D -r -t tcpip://localhost:17000  virtual_DriveStack.rt-win64 -uri tcpip://localhost:17001

TIMEOUT /T 1

start "Game Script" python game.py

TIMEOUT /T 4

start "QDrone2 Navigator Script" python QDrone2_Navigator.py

start "QCar2 Navigator Script" python QCar2_Navigator.py