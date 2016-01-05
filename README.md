# GT-S5367_GB_Opensource.zip

At some point in time [Samsung's Open Source Release Center](http://opensource.samsung.com/)
had the package for the **GT-S5367 (Galaxy Young TV)** smartphone 
available in their website.

The zipfile was called `GT-S5367_GB_Opensource.zip`

Which I managed to grab at the time.

It's no longer up there, so here it is the original version.

Mostly unmodified at initial commit, except for sane permissions and line endings fixes.

Code compiles and boots using the `bcm21553_totoro_05_tv_defconfig` configuration plus two
extra Makefile variables need to be specified in the commandline (alongside the usual ARCH and CROSS_COMPILE):
`LTN_BUILD_LOCALE="LTN"` and `SEC_PROJECT="TOTORO"`.
