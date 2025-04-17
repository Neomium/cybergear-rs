use std::path::Path;

fn main() {
    let target = std::env::var("TARGET").unwrap();
    let lib_path = Path::new("./cybergearclib/include");

    let mut build = cc::Build::new();
    let build_dir = Path::new("cybergearlib");
    build.out_dir(build_dir);

    build.include("cybergearclib/include/libcybergear");
    build.files([
        "cybergearclib/src/cyber_gear_protocol.c",
        "cybergearclib/src/float16.c",
        "cybergearclib/src/utils.c",
    ]);
    build.include(lib_path);

    let target_mcu = if target.contains("esp32s3") {
        build.compiler("xtensa-esp32s3-elf-gcc");
        build.archiver("xtensa-esp32s3-elf-ar");
        println!("cargo:warning=building for esp32s3");
        target.as_str()
    } else if target.contains("esp32") {
        build.compiler("xtensa-esp32-elf-gcc");
        build.archiver("xtensa-esp32-elf-ar");
        println!("cargo:warning=building for esp32");
        target.as_str()
    } else {
        println!("cargo:warning=building for {}", target);
        target.as_str()
    };

    build.flag("-ffunction-sections");
    build.flag("-fdata-sections");
    build.flag("-O2");

    if target_mcu.contains("esp32") {
        build.flag("-mlongcalls");
        build.flag("-mtext-section-literals");
    }

    let target_lib = format!("libCyberGearProtocol_{}.a", target_mcu);

    if build_dir.join(Path::new(target_lib.as_str())).exists() {
        println!(
            "cargo:warning={} already exists, skipping build",
            target_lib
        );
    } else {
        println!(
            "cargo:warning=libCyberGearProtocol_{}.a does not exist, building",
            target_mcu
        );
    }

    let lib_name = {
        let out = format!("CyberGearProtocol_{}", target_mcu);
        out
    };

    build.compile(lib_name.as_str());

    println!(
        "cargo:warning=finished building using {:?}",
        build.get_compiler().path()
    );

    println!("cargo:rustc-link-lib=static={lib_name}");
    println!("cargo:rustc-link-search=native=cybergearlib");
}
