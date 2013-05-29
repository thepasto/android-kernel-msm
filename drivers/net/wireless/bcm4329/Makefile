# bcm4329
ifeq ($(CONFIG_MACH_ACER_A1),y)
# +SDIO_ISR_THREAD
# +CUSTOMER_HW
# -DHD_USE_STATIC_BUF
DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2     \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
	-DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
	-Wall -Wstrict-prototypes -Werror -DCUSTOMER_HW                       \
	-DMMC_SDIO_ABORT -DSOFTAP -DSDIO_ISR_THREAD                           \
	-DSET_RANDOM_MAC_SOFTAP                                               \
	-Idrivers/net/wireless/bcm4329 -Idrivers/net/wireless/bcm4329/include
else
DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2     \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
	-DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
	-Wall -Wstrict-prototypes -Werror -DOOB_INTR_ONLY -DCUSTOMER_HW       \
	-DDHD_USE_STATIC_BUF -DMMC_SDIO_ABORT -DSOFTAP                        \
	-DSET_RANDOM_MAC_SOFTAP                                               \
	-Idrivers/net/wireless/bcm4329 -Idrivers/net/wireless/bcm4329/include
endif

DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o \
	wl_iw.o siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o   \
	dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o    \
	bcmsdh_sdmmc_linux.o

ifeq ($(CONFIG_MACH_ACER_A1),y)
	obj-$(CONFIG_BCM4329) += dhd.o
	dhd-objs += $(DHDOFILES)
else
	obj-$(CONFIG_BCM4329) += bcm4329.o
	bcm4329-objs += $(DHDOFILES)
endif

EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
