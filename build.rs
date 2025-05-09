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
        let cc = std::env::var("CC_XTENSA_ESP32S3_NONE_ELF")
            .unwrap_or_else(|_| "xtensa-esp32s3-elf-gcc".into());
        let ar = std::env::var("AR_XTENSA_ESP32S3_NONE_ELF")
            .unwrap_or_else(|_| "xtensa-esp32s3-elf-ar".into());
        build.compiler(cc.as_str());
        build.archiver(ar.as_str());
        println!("cargo:warning=building for esp32s3");
        target.as_str()
    } else if target.contains("esp32") {
        let cc = std::env::var("CC_XTENSA_ESP32_NONE_ELF")
            .unwrap_or_else(|_| "xtensa-esp32-elf-gcc".into());
        let ar = std::env::var("AR_XTENSA_ESP32_NONE_ELF")
            .unwrap_or_else(|_| "xtensa-esp32-elf-ar".into());
        build.compiler(cc.as_str());
        build.archiver(ar.as_str());
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
        println!("cargo:warning={target_lib} already exists, skipping build");
    } else {
        println!("cargo:warning=libCyberGearProtocol_{target_mcu}.a does not exist, building",);
    }

    let lib_name = {
        let out = format!("CyberGearProtocol_{target_mcu}");
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
