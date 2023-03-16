#include <ifm3d/device.h>
#include <gtest/gtest.h>
#include <fmt/core.h>
#include <optional>

// clang-format off
static std::vector<std::pair<std::string, ifm3d::SemVer>>
  VALID_VERSION_STRINGS{
    {"0.0.4", ifm3d::SemVer(0, 0, 4, std::nullopt, std::nullopt)},
    {"1.2.3", ifm3d::SemVer(1, 2, 3, std::nullopt, std::nullopt)},
    {"10.20.30", ifm3d::SemVer(10, 20, 30, std::nullopt, std::nullopt)},
    {"1.1.2-prerelease+meta", ifm3d::SemVer(1, 1, 2, "prerelease", "meta")},
    {"1.1.2+meta", ifm3d::SemVer(1, 1, 2, std::nullopt, "meta")},
    {"1.1.2+meta-valid", ifm3d::SemVer(1, 1, 2, std::nullopt, "meta-valid")},
    {"1.0.0-alpha", ifm3d::SemVer(1, 0, 0, "alpha", std::nullopt)},
    {"1.0.0-beta", ifm3d::SemVer(1, 0, 0, "beta", std::nullopt)},
    {"1.0.0-alpha.beta", ifm3d::SemVer(1, 0, 0, "alpha.beta", std::nullopt)},
    {"1.0.0-alpha.beta.1", ifm3d::SemVer(1, 0, 0, "alpha.beta.1", std::nullopt)},
    {"1.0.0-alpha.1", ifm3d::SemVer(1, 0, 0, "alpha.1", std::nullopt)},
    {"1.0.0-alpha0.valid", ifm3d::SemVer(1, 0, 0, "alpha0.valid", std::nullopt)},
    {"1.0.0-alpha.0valid", ifm3d::SemVer(1, 0, 0, "alpha.0valid", std::nullopt)},
    {"1.0.0-alpha-a.b-c-somethinglong+build.1-aef.1-its-okay", ifm3d::SemVer(1, 0, 0, "alpha-a.b-c-somethinglong", "build.1-aef.1-its-okay")},
    {"1.0.0-rc.1+build.1", ifm3d::SemVer(1, 0, 0, "rc.1", "build.1")},
    {"2.0.0-rc.1+build.123", ifm3d::SemVer(2, 0, 0, "rc.1", "build.123")},
    {"1.2.3-beta", ifm3d::SemVer(1, 2, 3, "beta", std::nullopt)},
    {"10.2.3-DEV-SNAPSHOT", ifm3d::SemVer(10, 2, 3, "DEV-SNAPSHOT", std::nullopt)},
    {"1.2.3-SNAPSHOT-123", ifm3d::SemVer(1, 2, 3, "SNAPSHOT-123", std::nullopt)},
    {"1.0.0", ifm3d::SemVer(1, 0, 0, std::nullopt, std::nullopt)},
    {"2.0.0", ifm3d::SemVer(2, 0, 0, std::nullopt, std::nullopt)},
    {"1.1.7", ifm3d::SemVer(1, 1, 7, std::nullopt, std::nullopt)},
    {"2.0.0+build.1848", ifm3d::SemVer(2, 0, 0, std::nullopt, "build.1848")},
    {"2.0.1-alpha.1227", ifm3d::SemVer(2, 0, 1, "alpha.1227", std::nullopt)},
    {"1.0.0-alpha+beta", ifm3d::SemVer(1, 0, 0, "alpha", "beta")},
    {"1.2.3----RC-SNAPSHOT.12.9.1--.12+788", ifm3d::SemVer(1, 2, 3, "---RC-SNAPSHOT.12.9.1--.12", std::nullopt)},
    {"1.2.3----R-S.12.9.1--.12+meta", ifm3d::SemVer(1, 2, 3, "---R-S.12.9.1--.12", std::nullopt)},
    {"1.2.3----RC-SNAPSHOT.12.9.1--.12", ifm3d::SemVer(1, 2, 3, "---RC-SNAPSHOT.12.9.1--.12", std::nullopt)},
    {"1.0.0+0.build.1-rc.10000aaa-kk-0.1", ifm3d::SemVer(1, 0, 0, std::nullopt, "0.build.1-rc.10000aaa-kk-0.1")},
    {"1.0.0-0A.is.legal", ifm3d::SemVer(1, 0, 0, "0A.is.legal", std::nullopt)},
  };
// clang-format on

static std::vector<std::string> INVALID_VERSION_STRINGS{
  "1",
  "1.2",
  "1.2.3-0123",
  "1.2.3-0123.0123",
  "1.1.2+.123",
  "+invalid",
  "-invalid",
  "-invalid+invalid",
  "-invalid.01",
  "alpha",
  "alpha.beta",
  "alpha.beta.1",
  "alpha.1",
  "alpha+beta",
  "alpha_beta",
  "alpha.",
  "alpha..",
  "beta",
  "1.0.0-alpha_beta",
  "-alpha.",
  "1.0.0-alpha..",
  "1.0.0-alpha..1",
  "1.0.0-alpha...1",
  "1.0.0-alpha....1",
  "1.0.0-alpha.....1",
  "1.0.0-alpha......1",
  "1.0.0-alpha.......1",
  "01.1.1",
  "1.01.1",
  "1.1.01",
  "1.2",
  "1.2.3.DEV",
  "1.2-SNAPSHOT",
  "1.2.31.2.3----RC-SNAPSHOT.12.09.1--..12+788",
  "1.2-RC-SNAPSHOT",
  "-1.0.3-gamma+b7718",
  "+justmeta",
  "9.8.7+meta+meta",
  "9.8.7-whatever+meta+meta",
  "99999999999999999999999.999999999999999999."
  "99999999999999999----RC-SNAPSHOT.12.09.1-----------"
  "---------------------..12",
};

static std::vector<std::tuple<std::string, std::string, int, bool>>
  COMPARE_VERSIONS{
    {"1.0.0", "1.0.0", 0, true},
    {"1.0.0-prerelease", "1.0.0", 0, false},
    {"1.0.0-prerelease+meta", "1.0.0", 0, false},
    {"1.0.0+meta", "1.0.0", 0, false},
    {"1.1.0", "1.0.0", +1, false},
    {"1.0.1", "1.0.0", +1, false},
    {"1.1.0-prerelease", "1.0.5", +1, false},
  };

TEST(Version, Version)
{
  int major, minor, patch;
  std::string tweak;
  std::string meta;
  ifm3d::version(&major, &minor, &patch, tweak, meta);
  EXPECT_EQ(IFM3D_VERSION,
            IFM3D_MAKE_VERSION(major, minor, patch, tweak, meta));
}

TEST(Version, ParseVersionValid)
{
  for (const auto& test_case : VALID_VERSION_STRINGS)
    {
      const auto [version_string, expected] = test_case;
      const auto tag = fmt::format("({0})", version_string);

      const auto parsed = ifm3d::SemVer::Parse(version_string);

      EXPECT_TRUE(parsed.has_value()) << tag;

      const auto actual = parsed.value();

      EXPECT_EQ(expected.major_num, actual.major_num) << tag;
      EXPECT_EQ(expected.minor_num, actual.minor_num) << tag;
      EXPECT_EQ(expected.patch_num, actual.patch_num) << tag;

      EXPECT_EQ(expected.prerelease.has_value(), actual.prerelease.has_value())
        << tag;

      EXPECT_EQ(expected.prerelease.has_value(), actual.prerelease.has_value())
        << tag;

      if (expected.prerelease.has_value())
        {
          const auto expected_prerelease = expected.prerelease.value();
          const auto actual_prerelease = expected.prerelease.value();
          EXPECT_STREQ(expected_prerelease.c_str(), actual_prerelease.c_str())
            << tag;
        }

      if (expected.build_meta.has_value())
        {
          const auto expected_build_meta = expected.build_meta.value();
          const auto actual_build_meta = expected.build_meta.value();
          EXPECT_STREQ(expected_build_meta.c_str(), actual_build_meta.c_str())
            << tag;
        }
    }
}

TEST(Version, ParseVersionInvalid)
{
  for (const auto& version_string : INVALID_VERSION_STRINGS)
    {
      const auto parsed = ifm3d::SemVer::Parse(version_string);
      EXPECT_FALSE(parsed.has_value()) << "(" << version_string << ")";
    }
}

TEST(Version, CompareVersions)
{
  for (const auto& test_case : COMPARE_VERSIONS)
    {
      const auto [v1_string, v2_string, compare_order, equal] = test_case;
      const auto tag = fmt::format("({0}, {1})", v1_string, v2_string);

      const auto v1 = ifm3d::SemVer::Parse(v1_string).value();
      const auto v2 = ifm3d::SemVer::Parse(v2_string).value();

      EXPECT_EQ(equal, v1 == v2) << tag;

      if (compare_order < 0)
        {
          EXPECT_TRUE(v1 < v2) << tag;
          EXPECT_TRUE(v1 <= v2) << tag;
          EXPECT_FALSE(v1 > v2) << tag;
        }
      else if (compare_order > 0)
        {
          EXPECT_TRUE(v1 > v2) << tag;
          EXPECT_TRUE(v1 >= v2) << tag;
          EXPECT_FALSE(v1 < v2) << tag;
        }
      else
        {
          EXPECT_TRUE(v1 >= v2) << tag;
          EXPECT_TRUE(v1 <= v2) << tag;
          EXPECT_FALSE(v2 < v1) << tag;
          EXPECT_FALSE(v2 > v1) << tag;
        }
    }
}