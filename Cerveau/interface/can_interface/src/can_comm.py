#!/usr/bin/python3

import can
from cerveau.interface.can_interface.include.frames import FrameList
from cerveau.interface.can_interface.include.devices import DeviceList
from lib.args_lib.include.argumentable import Argumentable


# device: ID=0, name=all pour broadcast, ID=1, name=bbb pour la beagle, ID=2, name=stm pour le STMP et ID=3, name=Arduino pour l'Arduino

def main():
    interface = input("interface [can1] : ")
    if interface == "":
        interface = "can1"

    can.rc['interface'] = 'socketcan_ctypes'
    bus = can.Bus(interface)

    devices = DeviceList.parse_file("Cerveau/Interfaces/can_interface/data/devices.xml", "interface_description")
    frames = FrameList.parse_file("Cerveau/Interfaces/can_interface/data/frames.xml", "interface_description", context={"devices": devices})

    while True:
        frame_name = input("can frame to send : ")

        while frame_name not in frames.by_name:
            print("available frames :\n\t{}".format("\n\t".join(frames.by_name.keys())))
            frame_name = input("can frame to send : ")

        frame = frames.by_name[frame_name]

        values = Argumentable()
        for param in frame.params:
            val = None

            while val is None:
                try:
                    val = int(input("value for {} : ".format(param.name)))
                except ValueError:
                    print("value must be int")
            values.set(param.name, val)

        print("preparing frame")
        message = can.Message(
            data=frame.get_frame_data(values),
            arbitration_id=devices.by_name[frame.dest],
            is_extended_id=False,
            dlc=frame.size()
        )

        print("sending frame")
        bus.send(message)


if __name__ == '__main__':
    main()
