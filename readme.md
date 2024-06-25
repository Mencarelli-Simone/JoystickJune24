## Changes to apply to files after cubemx run for joystick
USB_DEVICE/App/usb_device.c
		line 56 add
			USBD_HandleTypeDef* MX_USB_DEVICE_getHandle(void){
		  			return &hUsbDeviceFS;
			}
USB_DEVICE/App/usb_device.h
		line 84 add
			USBD_HandleTypeDef*  MX_USB_DEVICE_getHandle(void);


copy tocopy content into middleeware hid