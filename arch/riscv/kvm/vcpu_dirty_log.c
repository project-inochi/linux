// SPDX-License-Identifier: GPL-2.0

#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/kvm_host.h>
#include <linux/mm.h>
#include <asm/csr.h>
#include <asm/kvm_mmu.h>
#include <asm/kvm_nacl.h>
#include <asm/kvm_vcpu_dirty_log.h>

int kvm_cpu_dirty_log_size(struct kvm *kvm)
{
	return kvm->arch.dirty_state.entry_size;
}

void kvm_riscv_dirty_log_init(struct kvm *kvm)
{
	struct kvm_dirty_state *dirty_state = &kvm->arch.dirty_state;

	dirty_state->buffer_size = 0;
	dirty_state->entry_size = 0;

	if (!riscv_isa_extension_available(NULL, SHDLT))
		return;

	dirty_state->buffer_size = PAGE_SIZE;
	dirty_state->entry_size = DIRTY_LOG_BUFFER_CAPACITY(dirty_state->buffer_size);
	dirty_state->buffer_order = get_order(dirty_state->buffer_size);
}

void kvm_riscv_vcpu_dirty_log_deinit(struct kvm_vcpu *vcpu)
{
	struct kvm_dirty_state *dirty_state = &vcpu->kvm->arch.dirty_state;
	struct kvm_vcpu_dirty_log *dirty_log = &vcpu->arch.dirty_log;

	if (!dirty_state->buffer_size)
		return;

	free_pages((unsigned long)dirty_log->buffer, dirty_state->buffer_order);
	dirty_log->buffer = NULL;
}

int kvm_riscv_vcpu_alloc_dirty_buffer(struct kvm_vcpu *vcpu, int size)
{
	struct kvm_dirty_state *dirty_state = &vcpu->kvm->arch.dirty_state;
	struct kvm_vcpu_dirty_log *dirty_log = &vcpu->arch.dirty_log;
	struct page* dirty_buffer;

	dirty_log->buffer = NULL;
	dirty_log->buffer_phys = 0;

	if (!dirty_state->buffer_size)
		return 0;

	dirty_buffer = alloc_pages(GFP_KERNEL, dirty_state->buffer_order);
	if (!dirty_buffer)
		return -ENOMEM;

	dirty_log->buffer = page_to_virt(dirty_buffer);
	dirty_log->buffer_phys = page_to_phys(dirty_buffer);

	return 0;
}

void kvm_riscv_vcpu_dirty_log_load(struct kvm_vcpu *vcpu)
{
	struct kvm_dirty_state *dirty_state = &vcpu->kvm->arch.dirty_state;
	struct kvm_vcpu_dirty_log *dirty_log = &vcpu->arch.dirty_log;
	unsigned long ctrl;

	if (!dirty_state->buffer_size)
		return;

	ctrl = FIELD_PREP(HGDLTCTL_PPN, dirty_log->buffer_phys >> PAGE_SHIFT) |
	       FIELD_PREP(HGDLTCTL_SIZE, dirty_state->buffer_order) |
	       HGDLTCTL_EN;

	ncsr_write(CSR_HGDLTCTL, ctrl);
	ncsr_write(CSR_HGDLTIDX, 0);
}

void kvm_riscv_vcpu_dirty_log_put(struct kvm_vcpu *vcpu)
{
	kvm_riscv_vcpu_flush_dirty_buffer(vcpu);
}

void kvm_riscv_vcpu_flush_dirty_buffer(struct kvm_vcpu *vcpu)
{
	struct kvm_dirty_state *dirty_state = &vcpu->kvm->arch.dirty_state;
	struct kvm_vcpu_dirty_log *dirty_log = &vcpu->arch.dirty_log;
	unsigned long size, i;

	if (!dirty_state->buffer_size)
		return;

	size = ncsr_read(CSR_HGDLTIDX);

	if (size == 0)
		return;

	for (i = 0; i < size; i++) {
		gfn_t gpa = READ_ONCE(dirty_log->buffer[i]);

		kvm_vcpu_mark_page_dirty(vcpu, gpa >> PAGE_SHIFT);
	}

	ncsr_write(CSR_HGDLTIDX, 0);
}
