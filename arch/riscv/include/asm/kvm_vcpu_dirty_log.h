/* SPDX-License-Identifier: GPL-2.0-only */


#ifndef __KVM_VCPU_RISCV_DIRTY_LOG_H
#define __KVM_VCPU_RISCV_DIRTY_LOG_H

#include <linux/kvm_types.h>
#include <linux/bits.h>
#include <asm/csr.h>

#define KVM_VCPU_DIRTY_LOG_BUFFER_MAX_SIZE		0x400000

#define DIRTY_LOG_ENTRY32_PFN		GENMASK(31, 12)
#define DIRTY_LOG_ENTRY64_PFN		GENMASK_ULL(55, 12)
#define DIRTY_LOG_ENTRY_PTE_SHIFT	2

#ifdef CONFIG_64BIT
#define DIRTY_LOG_ENTRY_PFN		DIRTY_LOG_ENTRY64_PFN
#define DIRTY_LOG_ENTRY_SIZE		8
#else
#define DIRTY_LOG_ENTRY_PFN		DIRTY_LOG_ENTRY32_PFN
#define DIRTY_LOG_ENTRY_SIZE		4
#endif

#define DIRTY_LOG_BUFFER_CAPACITY(size)	((size) / DIRTY_LOG_ENTRY_SIZE)

struct kvm_dirty_state {
	unsigned int entry_size;
	unsigned int buffer_order;
	unsigned int buffer_size;
};

struct kvm_vcpu_dirty_log_csr {
	unsigned long status;
};

struct kvm_vcpu_dirty_log {
	struct kvm_vcpu_dirty_log_csr	csr;

	unsigned long			*buffer;
	phys_addr_t			buffer_phys;
};

void kvm_riscv_dirty_log_init(struct kvm *kvm);
int kvm_riscv_vcpu_alloc_dirty_buffer(struct kvm_vcpu *vcpu, int size);
void kvm_riscv_vcpu_dirty_log_deinit(struct kvm_vcpu *vcpu);
void kvm_riscv_vcpu_dirty_log_load(struct kvm_vcpu *vcpu);
void kvm_riscv_vcpu_dirty_log_put(struct kvm_vcpu *vcpu);
void kvm_riscv_vcpu_flush_dirty_buffer(struct kvm_vcpu *vcpu);

#endif /* __KVM_VCPU_RISCV_DIRTY_LOG_H */
