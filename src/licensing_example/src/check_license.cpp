// Minimal example: check the MoveIt Pro license from a user-workspace binary.
//
// Demonstrates linking a plain executable against the MoveIt Pro licensing
// library and querying license state via the public API in
// <moveit_studio_licensing/licensing.hpp>.

#include <iostream>

#include <moveit_studio_licensing/licensing.hpp>

int main()
{
  // Initialize the license cache up front. isValid()/getLicense() would do this
  // lazily on first call, but the check can involve a network round-trip, so
  // initializing explicitly keeps that latency out of later hot paths.
  moveit_studio::licensing::init();

  const bool valid = moveit_studio::licensing::isValid();
  const moveit_studio::licensing::License license = moveit_studio::licensing::getLicense();

  std::cout << "License valid: " << std::boolalpha << valid << '\n';
  std::cout << moveit_studio::licensing::describeLicenseStatus(license) << '\n';

  moveit_studio::licensing::shutdown();

  // Exit non-zero when unlicensed so this can gate a script or CI step.
  return valid ? 0 : 1;
}
