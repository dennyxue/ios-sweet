#!/usr/bin/env python3
"""
Build the Apple Device Tree for the OCEAN xnu (Redmi Note 2S / SDM710).

This is ios-sweet's tools/mkadt.py adapted to the Smartisan Nut Pro 2S.
The SWEET kernel is reused as-is: the SDM710 and the SM7150 agree on
everything the kernel reads at build time (ARMv8.2 without PAC, 16 KiB
pages, PAN, GICv3 with a 0x20000 redistributor stride, a 19.2 MHz generic
timer and a virtual timer INTID of 19), and the GIC even sits at the same
address, 0x17a00000.  What differs is all runtime data, which the device
tree supplies:

  * the CPU topology: the SDM710 has four Cortex-A75 (MPIDR 0x100-0x103)
    and four Cortex-A55 (MPIDR 0x0-0x3), where sweet has two A76 and six
    A55
  * the panel: 1080x2160 OLED, where sweet has 1080x2400
  * the storage: the eMMC host controller at 0x7c4000, where sweet has
    the UFS host at 0x1d84000

Usage:
    python3 tools/mkadt_ocean.py OUT.adt [--highmem] [--dump]

--highmem uses the upper DDR bank, which is what the phone wants: every
fixed Qualcomm carve-out on this SoC lives in the low bank.
"""

import hashlib
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import adt  # noqa: E402

# --- memory -------------------------------------------------------
# Kernel-managed RAM: above every Qualcomm XPU carve-out.
PHYS_BASE = 0xb0000000
MEM_SIZE = 0x4d000000

# The upper DDR bank.  Every fixed reserved-memory carve-out on this SoC
# (hyp_region at 0x85700000, xbl_region, removed_region, camera_region,
# modem_region, ...) is below 0x63900000, so the upper bank is free.
HIGH_PHYS_BASE = 0x140000000
HIGH_MEM_SIZE = 0x100000000

# Panic log buffer, carved from the TOP of the dram range and exposed to
# XNU as /pram + /chosen/embedded-panic-log-size.
PANIC_LOG_SIZE = 0x100000

# --- interrupt controller ------------------------------------------
# Same address as on sweet: sdm670.dtsi has
#   intc: interrupt-controller@17a00000 {
#           reg = <0x17a00000 0x10000>,   /* GICD */
#                 <0x17a60000 0x100000>;  /* GICR * 8 */
GICD, GICD_SIZE = 0x17a00000, 0x10000
GICR, GICR_SIZE = 0x17a60000, 0x100000

# --- display ------------------------------------------------------
# The bootloader leaves the panel in video mode showing its splash
# buffer; the address is board specific and comes from the live FDT at
# runtime.  This is the default the tools use.
FB, FB_SIZE = 0x9c000000, 0x1700000

# The Nut Pro 2S panel is 1080x2160; iOS is told it has the iPhone X
# panel and the picture is centred on it.
FB_W, FB_H, FB_STRIDE = 1080, 2160, 1080 * 4
PANEL_W, PANEL_H = 1080, 2160

# --- storage ------------------------------------------------------
# The SDM710 eMMC is a Qualcomm SDHCI controller: sdhci@7c4000, with
# reg = <0x7C4000 0x1000> for the host controller window.
EMMC, EMMC_SIZE = 0x07c4000, 0x1000

TIMEBASE = 19200000

OCEAN_ECID = 0x4f4345414e000001  # 'OCEAN' + 1, synthetic

BOOT_MANIFEST_HASH = hashlib.sha384(b'ocean boot manifest').digest()

u32 = lambda *v: struct.pack('<%dI' % len(v), *v)
u64 = lambda *v: struct.pack('<%dQ' % len(v), *v)
cstr = lambda s: s.encode() + b'\0'


def node(name, props=(), children=()):
    n = adt.Node()
    n.props = [('name', 0, cstr(name))] + [(k, 0, v) for k, v in props]
    n.children = list(children)
    return n


def build(highmem=False):
    phys_base, mem_size = (HIGH_PHYS_BASE, HIGH_MEM_SIZE) if highmem else (PHYS_BASE, MEM_SIZE)
    # Carve the panic log off the top so the kernel never allocates over it.
    panic_base = phys_base + mem_size - PANIC_LOG_SIZE
    mem_size -= PANIC_LOG_SIZE

    phandle = iter(range(1, 1000))
    ph = lambda: ('AAPL,phandle', u32(next(phandle)))

    chosen = node('chosen', [
        ph(),
        ('dram-base', u64(phys_base)),
        ('dram-size', u64(mem_size)),
        ('boot-args', cstr('rd=emmcroot0 emmcroot=userdata')),
        ('firmware-version', cstr('iBoot-8787.38.10')),
        ('device-type', cstr('iPhone10,3')),
        ('target-type', cstr('D22')),
        ('product-type', cstr('iPhone10,3')),
        ('unique-chip-id', u64(OCEAN_ECID)),
        ('chip-id', u32(0x8015)),
        ('certificate-production-status', u32(0)),
        ('effective-production-status-ap', u32(0)),
        ('certificate-security-mode', u32(1)),
        ('effective-security-mode-sep', u32(1)),
        ('boot-manifest-hash', u32(int.from_bytes(BOOT_MANIFEST_HASH[:4], 'little'))),
        ('embedded-panic-log-size', u32(PANIC_LOG_SIZE)),
    ])

    # No effaceable storage: keybagd then reads the system keybag as a
    # plain plist instead of decrypting it.
    defaults = node('defaults', [ph(), ('no-effaceable-storage', u32(1))])

    # --- CPUs ------------------------------------------------------
    # The SDM710 (Kryo 360) has:
    #   MPIDR 0x000-0x003  Cortex-A55 (Kryo 360 Silver), 1.7 GHz
    #   MPIDR 0x100-0x103  Cortex-A75 (Kryo 360 Gold),   2.2 GHz
    # XNU assigns logical ids by device tree order, so the list starts
    # with the boot core (MPIDR 0x0).  'reg' is the real MPIDR.
    mpidrs = [0x000, 0x001, 0x002, 0x003,
              0x100, 0x101, 0x102, 0x103]
    cpu_nodes = []
    for logical, mpidr in enumerate(mpidrs):
        gold = mpidr >= 0x100
        cpu_nodes.append(node(f'cpu{logical}', [
            ph(),
            ('device_type', cstr('cpu')),
            ('compatible', cstr('qcom,kryo360') + cstr('ARM,v8')),
            ('reg', u32(mpidr)),
            ('cpu-id', u32(logical)),
            ('cluster-id', u32(0)),
            ('cluster-type', u32(0)),
            ('die-cluster-id', u32(0)),
            ('cluster-core-id', u32(logical)),
            ('l2-cache-id', u32(logical)),
            ('l3-cache-id', u32(0)),
            ('state', cstr('running' if logical == 0 else 'stopped')),
            ('timebase-frequency', u32(TIMEBASE)),
            ('clock-frequency', u32(2200000000 if gold else 1700000000)),
            ('bus-frequency', u32(100000000)),
            ('memory-frequency', u32(1866000000)),
            ('peripheral-frequency', u32(TIMEBASE)),
            ('fixed-frequency', u32(TIMEBASE)),
        ]))
    cpus = node('cpus', [ph(), ('#address-cells', u32(1)),
                         ('#size-cells', u32(0))], cpu_nodes)

    gic = node('gic', [
        ph(),
        ('compatible', cstr('arm,gic-v3')),
        ('#interrupt-cells', u32(3)),
        ('interrupt-controller', b''),
        ('#redistributor-regions', u32(1)),
        ('redistributor-stride', u32(0x20000)),
        ('reg', u64(GICD, GICD_SIZE) + u64(GICR, GICR_SIZE)),
        ('interrupts', u32(1) + u32(9) + u32(4)),
    ])

    timer = node('timer', [
        ph(),
        ('compatible', cstr('arm,armv8-timer')),
        ('interrupt-controller', b''),
        ('interrupts', u32(1) + u32(1) + u32(0xf08) +
                       u32(1) + u32(2) + u32(0xf08) +
                       u32(1) + u32(3) + u32(0xf08) +
                       u32(1) + u32(0) + u32(0xf08)),
        ('clock-frequency', u32(TIMEBASE)),
    ])

    # --- display ----------------------------------------------------
    fbcon = node('fbcon', [
        ph(),
        ('compatible', cstr('arm,fbcon')),
        ('reg', u64(FB, FB_SIZE)),
        ('width', u32(PANEL_W)),
        ('height', u32(PANEL_H)),
        ('stride', u32(FB_STRIDE)),
    ])

    # --- storage ----------------------------------------------------
    # bsd/dev/sweet_emmc.c reads this node to find the host controller.
    emmc = node('sweet-emmc@7c4000', [
        ph(),
        ('compatible', cstr('qcom,sdhci-msm')),
        ('reg', u64(EMMC, EMMC_SIZE)),
    ])

    # MobileGestalt HomeButtonType = 2 (no Home button, gesture UI).
    buttons = node('buttons', [
        ph(),
        ('button-names', cstr('volup') + cstr('voldown') + cstr('hold')),
        ('home-button-type', u32(2)),
        ('opposed-power-vol-buttons', u32(1)),
    ])

    pram = node('pram', [
        ph(),
        ('reg', u64(panic_base, PANIC_LOG_SIZE)),
    ])

    return node('', [
        ph(),
        ('#address-cells', u32(2)),
        ('#size-cells', u32(2)),
    ], [chosen, defaults, cpus, gic, timer, fbcon, buttons, pram, emmc])


def main():
    if len(sys.argv) < 2:
        sys.exit(f"usage: {sys.argv[0]} OUT.adt [--highmem] [--dump]")

    out = sys.argv[1]
    highmem = '--highmem' in sys.argv
    root = build(highmem=highmem)
    blob = adt.serialize(root)
    open(out, 'wb').write(blob)
    if '--dump' in sys.argv:
        adt.dump(adt.parse(blob)[0])
    print(f'{out}: {len(blob)} bytes ({"highmem" if highmem else "lowmem"})')


if __name__ == '__main__':
    main()
