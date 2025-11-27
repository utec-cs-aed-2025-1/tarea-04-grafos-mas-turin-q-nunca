{
  pkgs ? import <nixpkgs> { },
}:
pkgs.mkShell {
  inputsFrom = [ (pkgs.callPackage ./. { }) ];

  packages = with pkgs; [
    valgrind
  ];
}
