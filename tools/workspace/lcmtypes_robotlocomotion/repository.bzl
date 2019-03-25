# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcmtypes_robotlocomotion_repository(
        name,
        mirrors = None):
    github_archive(
        name = "lcmtypes_robotlocomotion",
        repository = "sammy-tri/lcmtypes",
        commit = "d750ff71e21f2c424cb119d4a184af75f8ad82ac",
        sha256 = "42e8dc7a86e01ae671f0d07721a73478a220d65ed70fd1f3494d9f0c953596e8",  # noqa
        build_file = "@drake//tools/workspace/lcmtypes_robotlocomotion:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
