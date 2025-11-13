{
  pkgs ? import <nixpkgs> { },
}:
pkgs.stdenv.mkDerivation {
  pname = "aed-grafos";
  version = "main";

  src = ./.;

  nativeBuildInputs = with pkgs; [
    cmake
    sfml_2
  ];

  enableParallelBuilding = true;
}
