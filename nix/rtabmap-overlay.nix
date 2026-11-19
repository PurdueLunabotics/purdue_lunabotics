final: prev:
{
  rtabmap-conversions = final.callPackage ./rtabmap-conversions.nix {};
  rtabmap-costmap-plugins = final.callPackage ./rtabmap-costmap-plugins.nix {};
  rtabmap-demos = final.callPackage ./rtabmap-demos.nix {};
  rtabmap-examples = final.callPackage ./rtabmap-examples.nix {};
  rtabmap-launch = final.callPackage ./rtabmap-launch.nix {};
  rtabmap-msgs = final.callPackage ./rtabmap-msgs.nix {};
  rtabmap-odom = final.callPackage ./rtabmap-odom.nix {};
  rtabmap-python = final.callPackage ./rtabmap-python.nix {};
  rtabmap-ros = final.callPackage ./rtabmap-ros.nix {rtabmap-demos = null; rtabmap-examples = null;};
  rtabmap-rviz-plugins = final.callPackage ./rtabmap-rviz-plugins.nix {};
  rtabmap-slam = final.callPackage ./rtabmap-slam.nix {};
  rtabmap-sync = final.callPackage ./rtabmap-sync.nix {};
  rtabmap-util = final.callPackage ./rtabmap-util.nix {};
  rtabmap-viz = final.callPackage ./rtabmap-viz.nix {};
}
