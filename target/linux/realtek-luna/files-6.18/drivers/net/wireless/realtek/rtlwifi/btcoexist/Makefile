# SPDX-License-Identifier: GPL-2.0
btcoexist-y :=		halbtcoutsrc.o rtl_btc.o
btcoexist-$(CONFIG_RTL8192EE) += halbtc8192e2ant.o
btcoexist-$(CONFIG_RTL8723BE) += halbtc8723b1ant.o halbtc8723b2ant.o
btcoexist-$(CONFIG_RTL8821AE) += halbtc8821a1ant.o halbtc8821a2ant.o

obj-$(CONFIG_RTLBTCOEXIST) += btcoexist.o
