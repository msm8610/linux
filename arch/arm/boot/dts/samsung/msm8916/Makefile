ifeq ($(CONFIG_OF),y)

ifeq ($(CONFIG_SEC_FORTUNA_PROJECT),y)
	ifeq ($(CONFIG_MACH_FORTUNA_EUR_OPEN),y)
		dtb-y += msm8916-sec-fortuna-eur-r00.dtb
		dtb-y += msm8916-sec-fortuna-eur-r01.dtb
		dtb-y += msm8916-sec-fortuna-eur-r02.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA3G_LTN_DTV),y)
		dtb-y += msm8916-sec-fortunadtv-r00.dtb
		dtb-y += msm8916-sec-fortunadtv-r01.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA3G_EUR),y)
		dtb-y += msm8916-sec-fortuna-r00.dtb
		dtb-y += msm8916-sec-fortuna-r01.dtb
		dtb-y += msm8916-sec-fortuna-r03.dtb
		dtb-y += msm8916-sec-fortuna-r04.dtb
		dtb-y += msm8916-sec-fortuna-r06.dtb
		dtb-y += msm8916-sec-fortuna-r07.dtb
		dtb-y += msm8916-sec-fortuna-r09.dtb
		dtb-y += msm8916-sec-fortuna-r11.dtb
	else ifeq ($(CONFIG_MACH_FORTUNAVE3G_EUR),y)
		dtb-y += msm8916-sec-fortuna-ve3g-r07.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_SEA_OPEN),y)
		dtb-y += msm8916-sec-fortuna-sea-r07.dtb
		dtb-y += msm8916-sec-fortuna-sea-r08.dtb
		dtb-y += msm8916-sec-fortuna-sea-r09.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_TFN),y)
		dtb-y += msm8916-sec-fortuna-tfn-r00.dtb
		dtb-y += msm8916-sec-fortuna-tfn-r02.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_USC),y)
		dtb-y += msm8916-sec-fortuna-usc-r00.dtb
		dtb-y += msm8916-sec-fortuna-usc-r01.dtb
		dtb-y += msm8916-sec-fortuna-usc-r03.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_TMO),y)
                dtb-y += msm8916-sec-fortuna-tmo-r00.dtb
                dtb-y += msm8916-sec-fortuna-tmo-r02.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_SPR),y)
                dtb-y += msm8916-sec-fortuna-spr-r00.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_ACG),y)
                dtb-y += msm8916-sec-fortuna-acg-r00.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_CMCC),y)
		dtb-y += msm8916-sec-fortuna-r00.dtb
		dtb-y += msm8916-sec-fortuna-r01.dtb
		dtb-y += msm8916-sec-fortuna-r03.dtb
		dtb-y += msm8916-sec-fortuna-r04.dtb
		dtb-y += msm8916-sec-fortuna-r06.dtb
		dtb-y += msm8916-sec-fortuna-r07.dtb
		dtb-y += msm8916-sec-fortuna-r09.dtb
		dtb-y += msm8916-sec-fortuna-r10.dtb
		dtb-y += msm8916-sec-fortuna-r11.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA_CHN_ZT),y)
		dtb-y += msm8916-sec-fortuna-r00.dtb
		dtb-y += msm8916-sec-fortuna-r01.dtb
		dtb-y += msm8916-sec-fortuna-r03.dtb
		dtb-y += msm8916-sec-fortuna-r04.dtb
		dtb-y += msm8916-sec-fortuna-r06.dtb
		dtb-y += msm8916-sec-fortuna-r07.dtb
		dtb-y += msm8916-sec-fortuna-r09.dtb
		dtb-y += msm8916-sec-fortuna-r10.dtb
		dtb-y += msm8916-sec-fortuna-r11.dtb
	else ifeq ($(CONFIG_MACH_FORTUNA),y)
		dtb-y += msm8916-sec-fortuna-r00.dtb
		dtb-y += msm8916-sec-fortuna-r01.dtb
		dtb-y += msm8916-sec-fortuna-r03.dtb
		dtb-y += msm8916-sec-fortuna-r04.dtb
		dtb-y += msm8916-sec-fortuna-r06.dtb
		dtb-y += msm8916-sec-fortuna-r07.dtb
		dtb-y += msm8916-sec-fortuna-r09.dtb
		dtb-y += msm8916-sec-fortuna-r10.dtb
		dtb-y += msm8916-sec-fortuna-r11.dtb
        endif

endif

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../../,$(DTB_LIST))

clean-files := *.dtb

