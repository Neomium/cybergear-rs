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
        println!("cargo:warning=building for esp32s3 target");
        "esp32s3"
    } else if target.contains("esp32") {
        build.compiler("xtensa-esp32-elf-gcc");
        build.archiver("xtensa-esp32-elf-ar");
        println!("cargo:warning=building for esp32 target");
        "esp32"
    } else {
        ""
    };

    if build_dir.join("libCyberGearProtocol_esp32s3.a").exists() && target_mcu == "esp32s3" {
        println!("cargo:warning=libCyberGearProtocol_esp32s3.a already exists, skipping build");
        return;
    } else if build_dir.join("libCyberGearProtocol_esp32.a").exists() && target_mcu == "esp32" {
        println!("cargo:warning=libCyberGearProtocol_esp32.a already exists, skipping build");
        return;
    }

    let lib_name = if target.contains("esp32s3") {
        "CyberGearProtocol_esp32s3"
    } else {
        "CyberGearProtocol_esp32"
    };

    build.compile(lib_name);

    println!(
        "cargo:warning=finished building using {:?}",
        build.get_compiler()
    );

    println!("cargo:rustc-link-lib=static={lib_name}");
    println!("cargo:rustc-link-search=native=cybergearlib");
}
