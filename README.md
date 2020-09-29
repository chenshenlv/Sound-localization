# Museum Localization

## Important Things

* **NEVER** push to the master branch on main repository directly. Instead, go through the Following workflow EVERY TIME you want to add something to the main repository: `Fork->Clone->Edit->Commit->Pull->Push->Pull Request`
* Ask questions if you don't know something.

## Essensial Library Installation

### Sounddevice and Dependencies:

Install CFFI lib:
```
sudo apt-get install libffi-dev
```
Install Pysoundfile:
```
python3 -m pip install pysoundfile
sudo apt-get install libsndfile1
```
Install Pyaudio:
```
sudo apt-get install libasound2-dev
sudo apt-get install portaudio19-dev
pip install pyaudio --user
sudo apt-get install python3-pyaudio
```
Install Sounddevice:
```
python3 -m pip install sounddevice
```

### Marvelmind Dependency:
```
sudo apt-get install python-pip
sudo apt-get update
sudo apt-get install python-dev
sudo pip install crcmod
python -m pip install pyserial
```

### PulseAudio:
```
sudo apt-get install pulseaudio
```


## Quick Links

* [Documents](./Documentation)
* [Assignments](./Assignments)
* [Scrum and Other Notes](./Notes)
* [Archived MATLAB Code from 17 Spring](https://github.com/praenubilus/3D-Localization)
* [Archived Python Code from 18 Spring](https://github.com/shandysulen/3D-Audio-For-Museum-Exhibits)
* [Sensor Manufacture Website](https://marvelmind.com)
* [Offical Sample Code from Manufacture](https://github.com/MarvelmindRobotics/marvelmind.py)