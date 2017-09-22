# Thorlabs USB spectrometers CCS100, CCS125, CCS150, CCS175, CCS200

More info at https://www.thorlabs.com/thorproduct.cfm?partnumber=ccs200.

# Prerequisites

Before IOC can be started the _/dev/bus/usb/xxx/yyy_ needs to have proper permissions; for this the udev rules file _88-thorlabs-ccs.rules_ from _tlccsdrv_ EEE module can be copied to the _/etc/udev/rules.d_ folder.

# Usage

TODO

# Misc

When connected via USB a _/dev/bus/usb/xxx/yyy_ device node is created:

    [1225482.616929] usb 4-5: new high-speed USB device number 82 using xhci_hcd
    [1225482.745161] usb 4-5: New USB device found, idVendor=1313, idProduct=8088
    [1225482.745164] usb 4-5: New USB device strings: Mfr=0, Product=0, SerialNumber=0
    [1225483.352811] usb 4-5: USB disconnect, device number 82
    [1225486.860803] usb 4-5: new high-speed USB device number 83 using xhci_hcd
    [1225486.989424] usb 4-5: New USB device found, idVendor=1313, idProduct=8089
    [1225486.989427] usb 4-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
    [1225486.989429] usb 4-5: Product: CCS200
    [1225486.989430] usb 4-5: Manufacturer: Thorlabs
    [1225486.989431] usb 4-5: SerialNumber: M00414547
