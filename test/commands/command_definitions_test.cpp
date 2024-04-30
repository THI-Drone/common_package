#include <gtest/gtest.h>

#include <cinttypes>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_set>

#include "common_package/commands.hpp"

using namespace common_lib;

TEST(common_package, command_definition_test) {
    {
        // Test the definition of the "waypoint" command
        static const std::map<const std::string, const JsonKeyDefinition> def1 =
            CommandDefinitions::get_definition("waypoint");
        static const std::map<const std::string, const JsonKeyDefinition> def2 =
            CommandDefinitions::get_waypoint_command_definition();

        ASSERT_EQ(def1, def2);
    }
    {
        // Test the definition of the "detect_marker" command
        static const std::map<const std::string, const JsonKeyDefinition> def1 =
            CommandDefinitions::get_definition("detect_marker");
        static const std::map<const std::string, const JsonKeyDefinition> def2 =
            CommandDefinitions::get_detect_marker_command_definition();

        ASSERT_EQ(def1, def2);
    }
    {
        // Test the definition of the "drop_payload" command
        static const std::map<const std::string, const JsonKeyDefinition> def1 =
            CommandDefinitions::get_definition("drop_payload");
        static const std::map<const std::string, const JsonKeyDefinition> def2 =
            CommandDefinitions::get_drop_payload_command_definition();

        ASSERT_EQ(def1, def2);
    }
    {
        // Test the definition of the "end_mission" command
        static const std::map<const std::string, const JsonKeyDefinition> def1 =
            CommandDefinitions::get_definition("end_mission");
        static const std::map<const std::string, const JsonKeyDefinition> def2 =
            {};

        ASSERT_EQ(def1, def2);
    }
    {
        // Test the case where an invalid command is provided
        ASSERT_THROW(CommandDefinitions::get_definition("abc"),
                     std::runtime_error);
    }
}

TEST(common_package, parse_check_json_str_test) {
    // Checking that string parsing works

    {
        // Check invalid json string
        const std::string str = "invalid json";
        const std::map<const std::string, const JsonKeyDefinition> def = {
            {"key", JsonKeyDefinition(true, string)}};

        ASSERT_THROW(CommandDefinitions::parse_check_json_str(str, def),
                     std::runtime_error);
    }
    {
        // Check valid json string
        const std::string str = "{\"key\":\"valid json\"}";
        const std::map<const std::string, const JsonKeyDefinition> def = {
            {"key", JsonKeyDefinition(true, string)}};

        ASSERT_NO_THROW(CommandDefinitions::parse_check_json_str(str, def));
    }
}

TEST(common_package, parse_check_json_test) {
    // Key Checks

    {
        // Check that all keys are present and valid
        const nlohmann::json json = {{"null", nullptr},
                                     {"boolean", true},
                                     {"number", 123},
                                     {"number_float", 1.234},
                                     {"number_integer", -5},
                                     {"number_unsigned", 3u},
                                     {"string", "abc"},
                                     {"array", {1, 2, 3, 4, 5}},
                                     {"object", {{"one", 1}, {"two", 2}}}};

        const std::map<const std::string, const JsonKeyDefinition> def{
            {"null", {true, null}},
            {"boolean", {true, boolean}},
            {"number", {true, number}},
            {"number_float", {true, number_float}},
            {"number_integer", {true, number_integer}},
            {"number_unsigned", {true, number_unsigned}},
            {"string", {true, string}},
            {"array", {true, array}},
            {"object", {true, object}}};

        ASSERT_NO_THROW(CommandDefinitions::parse_check_json(json, def));
    }
    {
        // Check required and optional keys

        const std::map<const std::string, const JsonKeyDefinition> def{
            {"null", {true, null}},
            {"boolean", {false, boolean}},
            {"number", {true, number}},
            {"number_float", {false, number_float}},
            {"number_integer", {true, number_integer}},
            {"number_unsigned", {false, number_unsigned}},
            {"string", {true, string}},
            {"array", {false, array}},
            {"object", {true, object}}};

        {
            // All optional keys present
            const nlohmann::json json = {{"null", nullptr},
                                         {"boolean", true},
                                         {"number", 123},
                                         {"number_float", 1.234},
                                         {"number_integer", -5},
                                         {"number_unsigned", 3u},
                                         {"string", "abc"},
                                         {"array", {1, 2, 3, 4, 5}},
                                         {"object", {{"one", 1}, {"two", 2}}}};

            ASSERT_NO_THROW(CommandDefinitions::parse_check_json(json, def));
        }

        {
            // No optional keys present
            const nlohmann::json json = {{"null", nullptr},
                                         {"number", 123},
                                         {"number_integer", -5},
                                         {"string", "abc"},
                                         {"object", {{"one", 1}, {"two", 2}}}};

            ASSERT_NO_THROW(CommandDefinitions::parse_check_json(json, def));
        }

        {
            // One optional key present
            const nlohmann::json json = {
                {"null", nullptr}, {"boolean", true},
                {"number", 123},   {"number_integer", -5},
                {"string", "abc"}, {"object", {{"one", 1}, {"two", 2}}}};

            ASSERT_NO_THROW(CommandDefinitions::parse_check_json(json, def));
        }

        {
            // One required key missing
            const nlohmann::json json = {{"null", nullptr},
                                         {"boolean", true},
                                         {"number_float", 1.234},
                                         {"number_integer", -5},
                                         {"number_unsigned", 3u},
                                         {"string", "abc"},
                                         {"array", {1, 2, 3, 4, 5}},
                                         {"object", {{"one", 1}, {"two", 2}}}};

            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }

        {
            // One unknown key
            const nlohmann::json json = {{"null", nullptr},
                                         {"boolean", true},
                                         {"unknown_key", "this shouldn't be here"},
                                         {"number", 123},
                                         {"number_float", 1.234},
                                         {"number_integer", -5},
                                         {"number_unsigned", 3u},
                                         {"string", "abc"},
                                         {"array", {1, 2, 3, 4, 5}},
                                         {"object", {{"one", 1}, {"two", 2}}}};

            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }

        {
            // All keys missing
            const nlohmann::json json = {};

            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }
    }

    // Value checks
    {
        // Check that value is in bounds and of the correct type

        const std::map<const std::string, const JsonKeyDefinition> def{
            {"number_unsigned", {true, number_unsigned, 3, 6}}};

        {
            const nlohmann::json json = {{"number_unsigned", 3u}};
            ASSERT_NO_THROW(CommandDefinitions::parse_check_json(json, def));
        }
        {
            const nlohmann::json json = {{"number_unsigned", 2u}};
            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }
        {
            const nlohmann::json json = {{"number_unsigned", 3.0}};
            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }
        {
            const nlohmann::json json = {{"number_unsigned", 7u}};
            ASSERT_THROW(CommandDefinitions::parse_check_json(json, def), std::runtime_error);
        }
    }
}
