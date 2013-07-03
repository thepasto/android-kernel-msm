2.6.32.9 Kernel for Acer A1 (Liquid, S100)
==========================================

Based on [froyo_almond codeaurora][1] kernel branch snapshot as of
commit `5bfad4e19237da65af877d376727b2bef90b0950`.

Default configuration is in `arch/arm/configs/salsa_defconfig`.

Pre-built images based on Acer_LiquidE_4.008.08_EMEA_VFIT are available
[here][4].

See [froyo_almond on Acer Liquid E][2] blog post for more information.
This kernel will boot with stock Acer 2.2 userspace but will fail to
bring the Android UI - you will need updated lights.salsa, vold and
libaudio, libaudiopolicy, libacer_acoustic.so, TPA2018.csv, and keyboard
configuration from [Acer A1 device tree][3]. You will also
need libmedia, libmedia_jni.so, libaudioflinger.so from froyo_almond
codeaurora tree or from the pre-built images.

Upstream patches cherry-picked:

```
* bb9b6ef70f08f256ab4b8ec127c17ee629b85350
  leds: led-class.c - Quiet boot messages

* 14b5d6dd40b3091cb5f566568baa4a74dc619286
  leds: Fix race between LED device uevent and actual attributes creation

* 5a0e3ad6af8660be21ca98a971cd00f331318c05
  include cleanup: Update gfp.h and slab.h includes to prepare for
  breaking implicit slab.h inclusion from percpu.h

  [salsa] Updated drivers/leds/ledtrig-timer.c only

* 5ada28bf76752e33dce3d807bf0dfbe6d1b943ad
  led-class: always implement blinking

* 91facc22dec964683aef88f5620a790a6e46b98a
  led_class: fix typo in blink API
```

Upstream patches reverted:
```
 * 67dd178be5eb4a7ddb0c65bf6f87c66d1ba20359
   qsd8k: audio: Concurrency support for AAC encoding with Voice call

   [salsa] Guarded arch/arm/mach-msm/qdsp6/q6audio.c with #ifdef

 * Kgsl changes due to old userspace:
   ebb6af2, 5d2c9b1, ae30b91, 6b13aec, e950867, dc72bc2, 231994a, 3ec3994,
   9549b58, d7604ff, 2c21da7, 48f7784, 67120c8, 3d877ee, f0e9e1d, a4ff80e,
   59f1f1b, a440382, a845c6b

 * 112595d2baa0a127f3f4fef3995d8f20ad49a952
   [salsa] Reverted drivers/input/misc/gpio_input.c only

 * af279fc09a4144989298a61d974b7f081727ebd7
   [salsa] Guarded with #ifdefs CONFIG_MACH_ACER_A1

```

[1]: git://codeaurora.org/quic/la/kernel/msm
[2]: http://rtg.in.ua/2013/05/29/froyo_almond-on-acer-liquid-e/
[3]: https://github.com/roman-yepishev/android-device-acer-a1
[4]: http://yadi.sk/d/evctd6V25hnau
