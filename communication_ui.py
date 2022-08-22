from tkinter import *
from tkinter import messagebox
import tkinter as tk
import asyncio
import threading
from bleak import BleakScanner, BleakClient
import struct
##-------------------------------------------------------------------------
async def discover():
    devices = await BleakScanner.discover()

    for d in devices:
        print(d)


loop = asyncio.get_event_loop()
loop.run_until_complete(discover())

address = "6D:64:C0:2D:13:23" # set your address here

uuid_extend = lambda uuid_16: "0000" + uuid_16 + "-0000-1000-8000-00805f9b34fb"

def intermediate_temperature_callback(sender, data):
#    text.set("Emergency!! - Detected : Voice")   

    rx_data = format(struct.unpack('f', data[1:5])[0])

    if float(rx_data) > 15:
        text.set("Emergency!! - Detected : Voice + Gesture")   
    elif float(rx_data) < 15:
        text.set("Emergency!! - Detected : Only Gesture")   



    print("Received data in bytearray: {}".format(data))
    print("Received data in float: {}".format(struct.unpack('f', data[1:5])[0])) # Field struct is 5-byte long, where the first byte is the "flag" field.


def _asyncio_thread(async_loop):
    async_loop.run_until_complete(do_urls())


def do_tasks(async_loop):
    """ Button-Event-Handler starting the asyncio part. """
    threading.Thread(target=_asyncio_thread, args=(async_loop,)).start()


async def do_urls():
    print("Trying to connect...")
    async with BleakClient(address, use_cached=False) as client:
        print("Connected to {}".format(address))

        # For tutorial purpose -- list all services installed
        # Should print out "Generic Access Profile", "Generic Attribute Profile", and the user-installed standard service "Health Thermometer"
        services = await client.get_services()
        print("List of services..")
        for service in services:
            print(service)
        print("")

        print("Reading Generic Access Profile")
        device_name = await client.read_gatt_char(uuid_extend("2A00"))
        appearance = await client.read_gatt_char(uuid_extend("2A01"))
        print("Device name: {}".format(device_name))
        print("Appearance: {}".format(appearance))

        # Set the "notify" event handler
        # Handler will be called when the "peripheral"(server) "notify" the data
        await client.start_notify(uuid_extend("2A1E"), intermediate_temperature_callback)

        # Make the main function loop forever to continuously monitor the data
        while client.is_connected:
            await asyncio.sleep(1)


def do_freezed():
    messagebox.showinfo(message='Tkinter is reacting.')
    text.set("Text updated")        
    

def main(async_loop):
    #Button(master=root, text='Connect', command= lambda:do_tasks(async_loop)).pack()

    root.mainloop()

if __name__ == '__main__':
    root = Tk()
    root.geometry("500x250") 
    root.title("EE595 Project")
    Label_title = Label(master=root, text="EE595 Project - TEAM11",height=2, font=('Lato', 20, "bold")).pack()
    text = tk.StringVar()
    text.set("....")
    Label1 = Label(master=root, textvariable=text,height=2, font=('Lato', 15, "bold"), fg="red").pack()

    async_loop = asyncio.get_event_loop()
    threading.Thread(target=_asyncio_thread, args=(async_loop,)).start()
    main(async_loop)