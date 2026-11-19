{
  lib,
  fetchFromGitHub,
  buildPythonPackage,
  setuptools,
  wheel,
  torch,
  torchvision,
  numpy,
  opencv-python,
  matplotlib,
  kornia,
}:
buildPythonPackage {
  pname = "lightglue";
  version = "0.0";
  pyproject = true;

  src = fetchFromGitHub {
    owner = "cvg";
    repo = "LightGlue";
    rev = "5a650c444718caffc2669dc0094cde908f42dd73";
    hash = "sha256-LpYMMV7OcYKMRx23Pysh7Fc+JvXcrhdoDK0TuXLS67o=";
  };

  build-system = [
    setuptools
    wheel
  ];

  propagatedBuildInputs = [
    numpy
    torch
    torchvision
    opencv-python
    matplotlib
    kornia
  ];
}

