#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xfb080ece, "module_layout" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xc2959b64, "spi_register_controller" },
	{ 0x12585e08, "_raw_spin_unlock" },
	{ 0x556e4390, "clk_get_rate" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0x39a12ca7, "_raw_spin_unlock_irqrestore" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x7ed6139f, "_dev_err" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0xdb9ca3c5, "_raw_spin_lock" },
	{ 0x5f849a69, "_raw_spin_lock_irqsave" },
	{ 0x37a0cba, "kfree" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0x69cb4b41, "spi_finalize_current_message" },
	{ 0xc1514a3b, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "966E0589E62EFE5D23498EC");
