setup_astra_udev() {
    cp 56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules 
    echo "usb rules file install at /etc/udev/rules.d/56-orbbec-usb.rules" 
    sudo udevadm control --reload-rules && sudo udevadm trigger
}

main() {
    setup_astra_udev
}

main
