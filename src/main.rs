mod vm;

fn main() {
    let instructions = [0x67c9];
    let global_mem = [4];

    let entry = global_mem.len() * 4;

    let mut memory: Vec<u32> = vec![];
    memory.extend_from_slice(&global_mem);
    memory.extend_from_slice(&instructions);

    let mut vm = vm::Vm::new(
        bytemuck::cast_slice_mut(memory.as_mut_slice()),
        entry as u32,
    );

    vm.run().unwrap();
}
