#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
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
__used __section("__versions") = {
	{ 0x8e17b3ae, "idr_destroy" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x69acdf38, "memcpy" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x84323a64, "__tty_insert_flip_string_flags" },
	{ 0xf0a2251d, "tty_flip_buffer_push" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x20978fb9, "idr_find" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x9d217556, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x8da0819, "kmalloc_caches" },
	{ 0xd0c3484c, "kmalloc_trace" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x37a0cba, "kfree" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6d334118, "__get_user_8" },
	{ 0x7842f87, "usb_autopm_get_interface" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x5a4896a8, "__put_user_2" },
	{ 0x2fb51cc1, "usb_control_msg" },
	{ 0x44be8471, "pcpu_hot" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0x1000e51, "schedule" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0x7682ba4e, "__copy_overflow" },
	{ 0x7665a95b, "idr_remove" },
	{ 0x134d6c27, "usb_put_intf" },
	{ 0xff6339d5, "usb_deregister_dev" },
	{ 0x9d6b7b9d, "tty_port_tty_get" },
	{ 0x75fb2abd, "tty_vhangup" },
	{ 0x4a3c12a, "tty_kref_put" },
	{ 0xe25171e, "tty_unregister_device" },
	{ 0xb87fd47, "usb_free_urb" },
	{ 0x268c823c, "usb_driver_release_interface" },
	{ 0x6079cf62, "_dev_info" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xfb578fc5, "memset" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x539760ee, "tty_port_close" },
	{ 0x221fda33, "usb_ifnum_to_if" },
	{ 0xb8f11603, "idr_alloc" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x71da3ad, "usb_alloc_coherent" },
	{ 0x149d8909, "usb_alloc_urb" },
	{ 0x74d95011, "usb_driver_claim_interface" },
	{ 0x1ef84dda, "usb_get_intf" },
	{ 0xaa811051, "tty_port_register_device" },
	{ 0x7b873b3c, "tty_port_init" },
	{ 0xfd1890b1, "usb_register_dev" },
	{ 0x8f9c199c, "__get_user_2" },
	{ 0xcd9c13a3, "tty_termios_hw_change" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0x41ed3709, "get_random_bytes" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xe1827818, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xfcc619ff, "tty_register_driver" },
	{ 0xd2b432ed, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0x7c5d413a, "tty_unregister_driver" },
	{ 0x94dabf83, "tty_driver_kref_put" },
	{ 0xcf0b2268, "usb_bulk_msg" },
	{ 0xfe15742c, "_dev_err" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xb34512de, "usb_submit_urb" },
	{ 0xf29a0bac, "usb_autopm_put_interface_async" },
	{ 0x86e9e09c, "usb_autopm_get_interface_no_resume" },
	{ 0x14020d08, "usb_autopm_put_interface" },
	{ 0x94f6abae, "usb_get_from_anchor" },
	{ 0x5e255463, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0x6baa74ab, "usb_free_coherent" },
	{ 0xa4513dda, "tty_port_put" },
	{ 0xc9a0e42b, "usb_find_interface" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x3c079c21, "tty_port_tty_hangup" },
	{ 0xe2964344, "__wake_up" },
	{ 0xf8145464, "__dynamic_dev_dbg" },
	{ 0x62a4c39, "tty_port_tty_wakeup" },
	{ 0x819bf500, "tty_port_hangup" },
	{ 0x702f250b, "usb_autopm_get_interface_async" },
	{ 0x8cc355a9, "usb_anchor_urb" },
	{ 0xfc069fa7, "tty_port_open" },
	{ 0x23c074d9, "usb_deregister" },
	{ 0xe2fd41e5, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1A86pE018d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p55D9d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "53EDEFFEB22399681C397CD");
