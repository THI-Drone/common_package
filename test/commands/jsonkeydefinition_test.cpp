#include <gtest/gtest.h>

#include <cinttypes>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_set>

#include "common_package/commands.hpp"

using namespace common_lib;

TEST(common_package, json_key_definition_constructor_test) {
    {
        // Constructor with everything possible
        const std::unordered_set<data_type_t> every_data_type = {
            null,         boolean,        number,
            number_float, number_integer, number_unsigned,
            string,       array,          object};
        JsonKeyDefinition jsk(false, every_data_type, -10, 10);
        ASSERT_FALSE(jsk.required);
        ASSERT_EQ(jsk.data_types, every_data_type);
        ASSERT_EQ(jsk.min_val, -10);
        ASSERT_EQ(jsk.max_val, 10);
    }

    {
        // Invalid constructor
        const std::unordered_set<data_type_t> no_data_type = {};
        ASSERT_THROW(JsonKeyDefinition jsk(true, no_data_type),
                     std::runtime_error);
    }

    {
        // Constructor with minimum
        const std::unordered_set<data_type_t> data_types = {null};
        JsonKeyDefinition jsk(true, data_types);
        ASSERT_TRUE(jsk.required);
        ASSERT_EQ(jsk.data_types, data_types);
        ASSERT_FALSE(jsk.min_val.has_value());
        ASSERT_FALSE(jsk.max_val.has_value());
    }

    {
        // Constructor with min_val
        const std::unordered_set<data_type_t> data_types = {null};
        JsonKeyDefinition jsk(true, data_types, 3);
        ASSERT_TRUE(jsk.required);
        ASSERT_EQ(jsk.data_types, data_types);
        ASSERT_EQ(jsk.min_val, 3);
        ASSERT_FALSE(jsk.max_val.has_value());
    }

    {
        // Constructor with max_val
        const std::unordered_set<data_type_t> data_types = {null};
        JsonKeyDefinition jsk(true, data_types, std::nullopt, 4);
        ASSERT_TRUE(jsk.required);
        ASSERT_EQ(jsk.data_types, data_types);
        ASSERT_FALSE(jsk.min_val.has_value());
        ASSERT_EQ(jsk.max_val, 4);
    }

    {
        // Constructor with min_val > max_val
        const std::unordered_set<data_type_t> data_types = {null};
        JsonKeyDefinition jsk(true, data_types, 5, 4);
        ASSERT_TRUE(jsk.required);
        ASSERT_EQ(jsk.data_types, data_types);
        ASSERT_EQ(jsk.min_val, 4);
        ASSERT_EQ(jsk.max_val, 5);
    }

    {
        // Second constructor
        const std::unordered_set<data_type_t> data_types = {number};
        JsonKeyDefinition jsk(true, number);
        ASSERT_TRUE(jsk.required);
        ASSERT_EQ(jsk.data_types, data_types);
        ASSERT_FALSE(jsk.min_val.has_value());
        ASSERT_FALSE(jsk.max_val.has_value());
    }
}

TEST(common_package, json_key_definition_type_check_and_bounds_check) {
    {
        // Test with number data type and specified min and max values
        JsonKeyDefinition jsk(false, number, -10, 10);
        {
            // Test with valid integer value within the specified range
            nlohmann::json json = {5};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid integer value at the lower bound
            nlohmann::json json = {-10};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid integer value at the upper bound
            nlohmann::json json = {10};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid floating-point value within the specified range
            nlohmann::json json = {1.345};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with floating-point value outside the specified range
            nlohmann::json json = {10.00000001};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with integer value outside the specified range
            nlohmann::json json = {-11};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with string value (invalid type)
            nlohmann::json json = {"abc"};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with nested JSON object (invalid type)
            nlohmann::json json = {{"abc", {"def", 234}}};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
    }
    {
        // Test with number_float and number_unsigned data types and specified
        // min and max values
        JsonKeyDefinition jsk(
            false,
            std::unordered_set<data_type_t>{number_float, number_unsigned}, -10,
            10);
        {
            // Test with valid unsigned integer value within the specified range
            nlohmann::json json = {5u};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid floating-point value within the specified range
            nlohmann::json json = {0.234};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid integer value (outside the specified range)
            nlohmann::json json = {-1};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid unsigned integer value (outside the specified
            // range)
            nlohmann::json json = {11u};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid floating-point value (outside the specified
            // range)
            nlohmann::json json = {10.123};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid integer value (outside the specified range)
            nlohmann::json json = {-11};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
    }
    {
        // Test with number_float data type and specified min and max values
        JsonKeyDefinition jsk(
            false, std::unordered_set<data_type_t>{number_float}, -10, 10);
        {
            // Test with valid floating-point value within the specified range
            nlohmann::json json = {0.1234};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid floating-point value at the upper bound
            nlohmann::json json = {10.0};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid integer value (outside the specified range)
            nlohmann::json json = {-1};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
    }
    {
        // Test with number data type and specified min value
        JsonKeyDefinition jsk(false, number, -10);
        {
            // Test with invalid string value (invalid type)
            nlohmann::json json = {"abc"};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid floating-point value
            nlohmann::json json = {1234.0};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid integer value (outside the specified range)
            nlohmann::json json = {-11};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
    }
    {
        // Test with number data type and specified max value
        JsonKeyDefinition jsk(false, number, std::nullopt, 10);
        {
            // Test with invalid string value (invalid type)
            nlohmann::json json = {"abc"};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid negative floating-point value
            nlohmann::json json = {-1234.0};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with invalid integer value (outside the specified range)
            nlohmann::json json = {11};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_FALSE(jsk.check_bounds(json.begin()));
        }
    }
    {
        // Test with number data type (no min or max value specified)
        JsonKeyDefinition jsk(false, number);
        {
            // Test with invalid string value (invalid type)
            nlohmann::json json = {"abc"};
            ASSERT_FALSE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid negative floating-point value
            nlohmann::json json = {-1234.0};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with valid negative integer value
            nlohmann::json json = {-1234};
            ASSERT_TRUE(jsk.type_check(json.begin()));
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
    }

    // Test with all possible data types
    data_type_t data_types[] = {null,         boolean,        number,
                                number_float, number_integer, number_unsigned,
                                string,       array,          object};

    for (const auto& data_type : data_types) {
        JsonKeyDefinition jsk(false, data_type);

        {
            // Test with null value
            nlohmann::json json = {nullptr};
            if (data_type == null) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with boolean value
            nlohmann::json json = {true};
            if (data_type == boolean) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with integer value
            nlohmann::json json = {11};
            if (data_type == number || data_type == number_integer) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with floating-point value
            nlohmann::json json = {1.234};
            if (data_type == number_float || data_type == number) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with unsigned integer value
            nlohmann::json json = {3u};
            if (data_type == number_unsigned || data_type == number ||
                data_type == number_integer) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with string value
            nlohmann::json json = {"abc"};
            if (data_type == string) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with array value
            nlohmann::json json = {{1, 2, 4, 8, 16}, {1, 3, 4}};
            if (data_type == array) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
        {
            // Test with object value
            nlohmann::json json = {{{"key1", "value1"}}, {"key2", "value2"}};
            if (data_type == object) {
                ASSERT_TRUE(jsk.type_check(json.begin()));
            } else {
                ASSERT_FALSE(jsk.type_check(json.begin()));
            }
            ASSERT_TRUE(jsk.check_bounds(json.begin()));
        }
    }
}

TEST(common_package, json_key_definition_data_type_to_string) {
    // Test that the data_type_to_string function returns the correct string
    // representation for each valid data type
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(null), "null");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(boolean), "bool");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(number), "number");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(number_float), "float");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(number_integer), "int");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(number_unsigned), "uint");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(string), "string");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(array), "array");
    ASSERT_EQ(JsonKeyDefinition::data_type_to_string(object), "object");

    // Test that the data_type_to_string function throws a std::runtime_error
    // when an invalid data type is passed
    ASSERT_THROW(JsonKeyDefinition::data_type_to_string((data_type_t)-1),
                 std::runtime_error);
}
