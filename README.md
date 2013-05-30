Kernel version 2.6.32.9 for Acer Liquid E
=========================================

Based on [froyo_almond codeaurora][1] kernel branch snapshot as of
commit `5bfad4e19237da65af877d376727b2bef90b0950`.

See [froyo_almond on Acer Liquid E][2] blog post for more information.
This kernel will boot with stock Acer 2.2 userspace but will fail to
bring the Android UI - you will need the updated [liblights][3].

Default configuration is in `arch/arm/configs/salsa_defconfig`.

[1]: git://codeaurora.org/quic/la/kernel/msm
[2]: http://rtg.in.ua/2013/05/29/froyo_almond-on-acer-liquid-e/
[3]: https://github.com/roman-yepishev/acer_a1_device_acer_a1
