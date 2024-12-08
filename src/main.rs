mod vm;

fn main() {
    let mut instructions = [
        0x00001537 as u32,
        0x00000593,
        0x00B50663,
        0xFFF50513,
        0xFF9FF06F,
        0x0000006F,
    ];

    let mut vm = vm::Vm::new(bytemuck::cast_slice_mut(&mut instructions), 0);

    vm.run().unwrap();
}
