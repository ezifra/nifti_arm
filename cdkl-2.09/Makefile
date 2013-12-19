include Config

all: driver cpclib

# cpc.h is a file included by applications and kernel modules. Therfor
# it should be in $(KERNELDIR)/include/linux and in /usr/include/linux 
install_headers_h:
	@echo "Installing header files."
	@install -o root -g root -m 644 include/cpc.h $(CPC_H_PATH)
	@install -o root -g root -m 644 include/cpclib.h /usr/include

# driver ###################################################
driver: cpccard cpcpci cpcusb cpcxt cpceco
	
cpcpci:
ifeq ($(strip $(CPC_PCI_SUPPORTED)),true) 
	@echo "Building CPC-PCI driver."
	@$(MAKE) -f Makefile.custom -C cpc-pci driver > $(OUTPUT_FILE)
else
	@echo "CPC-PCI is not configured in Makefile. Skipping."
endif

cpcusb:
ifeq ($(strip $(CPC_USB_SUPPORTED)),true) 
	@echo "Building CPC-USB driver."
	@$(MAKE) -f Makefile.custom -C cpc-usb driver > $(OUTPUT_FILE)
else
	@echo "CPC-USB is not configured in Makefile. Skipping."
endif


cpccard:
ifeq ($(strip $(CPC_CARD_SUPPORTED)),true) 
	@echo "Building CPC-CARD driver."
	@$(MAKE) -f Makefile.custom -C cpc-card driver > $(OUTPUT_FILE)
else
	@echo "CPC-CARD is not configured in Makefile. Skipping."
endif

cpcxt:
ifeq ($(strip $(CPC_XT_SUPPORTED)),true)
	@echo "Building CPC-XT driver."
	@$(MAKE) -f Makefile.custom -C cpc-xt driver > $(OUTPUT_FILE)
else
	@echo "CPC-XT is not configured in Makefile. Skipping."
endif

cpceco:
ifeq ($(strip $(CPC_ECO_SUPPORTED)),true)
	@echo "Building CPC-ECO driver."
	@$(MAKE) -f Makefile.custom -C cpc-eco driver > $(OUTPUT_FILE)
else
	@echo "CPC-ECO is not configured in Makefile. Skipping."
endif

# install ##################################################
install: install_headers_h cpclib driver

ifeq ($(strip $(CPC_PCI_SUPPORTED)),true) 
	@echo "Installing CPC-PCI driver."
	@$(MAKE) -f Makefile.custom -C cpc-pci install > $(OUTPUT_FILE)
ifeq ($(strip $(CREATE_INODES)),true)
	@echo "Creating inodes for CPC-PCI driver."
	@$(MAKE) -f Makefile.custom -C cpc-pci inodes > $(OUTPUT_FILE)
endif
else
	@echo "CPC-PCI is not configured in Makefile. Skipping."
endif

ifeq ($(strip $(CPC_USB_SUPPORTED)),true)
	@echo "Installing CPC-USB driver."
	@$(MAKE) -f Makefile.custom -C cpc-usb install > $(OUTPUT_FILE)
ifeq ($(strip $(CREATE_INODES)),true)
	@echo "Creating inodes for CPC-USB driver."
	@$(MAKE) -f Makefile.custom -C cpc-usb inodes > $(OUTPUT_FILE)
endif
else 
	@echo "CPC-USB is not configured in Makefile. Skipping." 
endif

ifeq ($(strip $(CPC_CARD_SUPPORTED)),true) 
	@echo "Installing CPC-CARD driver."
	@$(MAKE) -f Makefile.custom -C cpc-card install > $(OUTPUT_FILE)
ifeq ($(strip $(CREATE_INODES)),true)
	@echo "Creating inodes for CPC-CARD driver."
	@$(MAKE) -f Makefile.custom -C cpc-card inodes > $(OUTPUT_FILE)
endif
else
	@echo "CPC-CARD is not configured in Makefile. Skipping."
endif

ifeq ($(strip $(CPC_XT_SUPPORTED)),true)
	@echo "Installing CPC-XT driver."
	@$(MAKE) -f Makefile.custom -C cpc-xt install > $(OUTPUT_FILE)
ifeq ($(strip $(CREATE_INODES)),true)
	@echo "Creating inodes for CPC-XT driver."
	@$(MAKE) -f Makefile.custom -C cpc-xt inodes > $(OUTPUT_FILE)
endif
else
	@echo "CPC-XT is not configured in Makefile. Skipping."
endif

ifeq ($(strip $(CPC_ECO_SUPPORTED)),true) 
	@echo "Installing CPC-ECO driver."
	@$(MAKE) -f Makefile.custom -C cpc-eco install > $(OUTPUT_FILE)
ifeq ($(strip $(CREATE_INODES)),true)
	@echo "Creating inodes for CPC-ECO driver."
	@$(MAKE) -f Makefile.custom -C cpc-eco inodes > $(OUTPUT_FILE)
endif
else
	@echo "CPC-ECO is not configured in Makefile. Skipping."
endif
	
	@[ ! -d $(UDEV_CONF_PATH)/rules.d ] || install -o root -g root -m 644 cpc.rules $(UDEV_CONF_PATH)/rules.d/

	@$(DEPMOD) -aq

	@echo "Installing docs"
	@$(MAKE) -C man install > $(OUTPUT_FILE)
	
	@echo "Installing CPCLIB"
	@$(MAKE) -C lib install > $(OUTPUT_FILE)

	@echo "-----------------------"
	@echo "To generate a appropriate cpcconf.ini, connect your devices and load all"
	@echo "drivers, for example:"
	@echo "       modprobe cpc-usb"
	@echo "       modprobe cpc-pci"
	@echo "       ..."
	@echo ""
	@echo "If all drivers loaded and all device are connected, please type: make generate_cpcconf"
	@echo "WARNING: This will overwrite any existing cpcconf.ini in /etc"

cpclib:
	@echo "Building CPCLIB."
	@$(MAKE) -C ethercan all > $(OUTPUT_FILE)
ifeq ($(strip $(SERIAL_SUPPORT)),true) 
	@$(MAKE) -C serial all > $(OUTPUT_FILE)
endif
	@$(MAKE) -C lib all > $(OUTPUT_FILE)
	
doc:
	@$(MAKE) man install > $(OUTPUT_FILE)
	
demo:	cpclib
	@echo "Building CPCTOOL."
	@$(MAKE) -C cpctool cpctool > $(OUTPUT_FILE)

generate_cpcconf:
	@echo "Generating new cpcconf.ini -> /etc/cpcconf.ini"
	@python tools/generate_cpcconf.py > /etc/cpcconf.ini
	@chmod 744 /etc/cpcconf.ini

inodes:
	@echo -n "Creating inodes in /dev ... "
	@$(MAKE) -f Makefile.custom -C cpc-pci inodes > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-eco inodes > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-xt inodes > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-usb inodes > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-card inodes > $(OUTPUT_FILE)
	@echo "done."

clean:
	@echo -n "Cleaning buildtree ... "
	@$(MAKE) -f Makefile.custom -C cpc-pci clean > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-eco clean > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-xt clean > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-usb clean > $(OUTPUT_FILE)
	@$(MAKE) -f Makefile.custom -C cpc-card clean > $(OUTPUT_FILE)
	@$(MAKE) -C lib clean > $(OUTPUT_FILE)
	@$(MAKE) -C ethercan clean > $(OUTPUT_FILE)
	@$(MAKE) -C serial clean > $(OUTPUT_FILE)
	@$(MAKE) -C cpctool clean > $(OUTPUT_FILE)
	@echo "done."
