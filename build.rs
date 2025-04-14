fn main() {
    println!("cargo:rustc-link-lib=static=CCyberGearProtocol");
    println!("cargo:rustc-link-search=native=cybergearlib");
}
