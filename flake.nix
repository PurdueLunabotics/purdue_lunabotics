{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nix-ros-overlay = {
      url = "github:lopsided98/nix-ros-overlay/master";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    rtabmap_ros = {
      url = "github:introlab/rtabmap_ros";
      flake = false;
    };
    rtabmap = {
      url = "github:introlab/rtabmap";
      flake = false;
    };
  };
  outputs = { self, nix-ros-overlay, nixpkgs, ... }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        applyDistroOverlay =
          rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
          ) rosPackages;
        rosDistroOverlays = final: prev: {
          # Apply the overlay to multiple ROS distributions
          rosPackages = applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rosDistroOverlays
            (final: prev: {
              rosPackages = applyDistroOverlay (import ./nix/rtabmap-overlay.nix) prev.rosPackages;
              rosSources = {
                inherit (inputs) rtabmap_ros;
              };
            })
            (final: prev: {
              pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
                (python-final: python-prev: {
                  lightglue = python-final.callPackage ./lightglue.nix { };
                })
              ];

              rosPackages = applyDistroOverlay (final_ros: prev_ros: {
                octomap = prev_ros.octomap.overrideAttrs (final_attrs: prev_attrs: {
                  patches = (prev_attrs.patches or []) ++ [ ./nix/octomap.patch ];
                });

                usb-cam = prev_ros.usb-cam.overrideAttrs (final_attrs: prev_attrs: {
                  patches = (prev_attrs.patches or []) ++ [ ./nix/usb-cam.patch ];
                });

                nav2-navfn-planner = prev_ros.nav2-navfn-planner.overrideAttrs (final_attrs: prev_attrs: {
                  patches = (prev_attrs.patches or []) ++ [ ./nix/navfn.patch ];
                });

                rtabmap = prev_ros.rtabmap.overrideAttrs (final_attrs: prev_attrs: {
                  src = inputs.rtabmap;
                  cmakeFlags = (prev_attrs.cmakeFlags or []) ++ [
                    "-DWITH_PYTHON=ON"
                  ];
                  buildInputs = (prev_attrs.buildInputs or []) ++ [
                    (final.python3.withPackages (ps: with ps; [ lightglue ]))
                  ];
                });

                # realsense2-camera = prev_ros.realsense2-camera.overrideAttrs (final_attrs: prev_attrs: {
                #   cmakeFlags = (prev_attrs.cmakeFlags or []) ++ [
                #     "-DUSE_LIFECYCLE_NODE=ON"
                #   ];
                # });
              }) prev.rosPackages;
            })
          ];
        };
        rosDistro = "jazzy";

      in {
        legacyPackages = pkgs.rosPackages;
        packages = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        checks = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        devShells.default = import ./shell.nix {
          inherit pkgs rosDistro;
          extraPkgs = {  };
          extraPaths = [
            (pkgs.python3.withPackages (ps: with ps; [ lightglue protobuf ]))
            pkgs.rosPackages.jazzy.usb-cam
            pkgs.rosPackages.jazzy.ffmpeg-image-transport
            pkgs.librsvg
            pkgs.gdk-pixbuf
            pkgs.wrapGAppsHook3
          ];
        };
      });

  nixConfig = {
    extra-substituters = [
      "ssh-ng://luna-nix"
    ];
  };
}
