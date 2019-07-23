from pyparrot.Minidrone import Mambo

mamboAddr = "d0:3a:b7:81:e6:3b"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)
