final: prev:
{
  lunabot-behavior = final.callPackage ./lunabot_behavior/package.nix {};
  lunabot-bringup = final.callPackage ./lunabot_bringup/package.nix {};
  lunabot-config = final.callPackage ./lunabot_config/package.nix {};
  lunabot-control = final.callPackage ./lunabot_control/package.nix {};
  lunabot-description = final.callPackage ./lunabot_description/package.nix {};
  lunabot-embedded = final.callPackage ./lunabot_embedded/package.nix {};
  lunabot-msgs = final.callPackage ./lunabot_msgs/package.nix {};
  lunabot-nav = final.callPackage ./lunabot_nav/package.nix {};
  lunabot-perception = final.callPackage ./lunabot_perception/package.nix {};
  lunabot-sim = final.callPackage ./lunabot_sim/package.nix {};
  lunabot-slam = final.callPackage ./lunabot_slam/package.nix {};
  test-firmware = final.callPackage ./test_firmware/package.nix {};
}
