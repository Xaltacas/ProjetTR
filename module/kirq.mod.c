#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0x2a51cf90, "module_layout" },
	{ 0x413cc4ae, "class_unregister" },
	{ 0xaff3d0c0, "device_destroy" },
	{ 0xedc03953, "iounmap" },
	{ 0xfe990052, "gpio_free" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xefc93a94, "class_destroy" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x6ebc8180, "device_create" },
	{ 0x359cafaf, "__class_create" },
	{ 0xe7ca5a52, "__register_chrdev" },
	{ 0xe97c4103, "ioremap" },
	{ 0xbc10dd97, "__put_user_4" },
	{ 0xc5850110, "printk" },
	{ 0xdbdb0e8b, "request_any_context_irq" },
	{ 0xb458687b, "gpiod_to_irq" },
	{ 0x7f18713d, "gpio_to_desc" },
	{ 0x403f9529, "gpio_request_one" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "3982DCAC2F72B98EF64DA3B");
