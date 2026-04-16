fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .build_server(false) // we only need the gRPC client
        .compile_protos(
            &["proto/kuksa/val/v1/val.proto"],
            &["proto"],
        )?;
    Ok(())
}
