#Makefile for xdr sources
#EXTRA_CFLAGS := -Werror

EXTRA_CFLAGS += -I drivers/staging/bcm21553/drivers/char/brcm/fuse_rpc/rpc_CIB/public/
EXTRA_CFLAGS += -I drivers/staging/bcm21553/drivers/char/brcm/fuse_rpc/rpc_CIB/arpc/inc/
#EXTRA_CFLAGS += -I drivers/staging/bcm21553/drivers/char/brcm/fuse_rpc/rpc_CIB/srpc/inc/

# common definitons for all capi2 related include paths
include drivers/staging/bcm21553/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2_include_paths.gmk

EXTRA_CFLAGS += -DUNDER_LINUX

obj-$(CONFIG_BRCM_FUSE_RPC_CIB) += \
	xdr.o \
	xdr_array.o \
	xdr_mem.o \
	xdr_float.o \
	xdr_reference.o\
