this is kernel module(and userland tool) to use
Allwinner's sunxi A64 spi in slave mode

it create char device /dev/spi\_mon and use SPI0 in slave mode
to monitor SPI traffic

use userland program rwspi.c to dump the data!

