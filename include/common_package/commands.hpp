#pragma once

#include <cinttypes>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace common_lib {
// Global Defines
constexpr uint16_t MAX_FLIGHT_HEIGHT_CM =
    120 /* [m] */ * 100;  /// Maximum allowed flight height in cm
constexpr uint16_t MIN_CRUISE_HEIGHT_CM =
    5 /* [m] */ * 100;  /// Minimum required cruise height in cm
constexpr float MAX_HORIZONTAL_SPEED_MPS =
    12.0 /* [m/s] */;  /// Maximum allowed horizontal speed in m/s
constexpr float MAX_VERTICAL_SPEED_MPS =
    3.0 /* [m/s] */;  /// Maximum allowed vertical speed in m/s

/**
 * @brief Enumeration representing the data types.
 */
typedef enum data_type {
    null,
    boolean,
    number,
    number_float,
    number_integer,
    number_unsigned,
    string,
} data_type_t;

struct JsonKeyDefinition {
    bool required;  /// If true, the value needs to be provided
    std::unordered_set<data_type_t>
        data_types;  // Unsorted set of accepted data types for the specific key
    std::optional<float>
        min_val;  // Min value the value should have (only works with numbers)
    std::optional<float>
        max_val;  // Max value the value should have (only works with numbers)

    /**
     * Converts a data_type_t value to its corresponding string representation.
     *
     * @param data_type The data_type_t value to convert.
     * @return The string representation of the data_type_t value.
     * @throws std::runtime_error if an unknown data_type is provided.
     */
    static std::string data_type_to_string(const data_type_t data_type) {
        switch (data_type) {
            case null:
                return "null";
            case boolean:
                return "bool";
            case number:
                return "number";
            case number_float:
                return "float";
            case number_integer:
                return "int";
            case number_unsigned:
                return "uint";
            default:
                throw std::runtime_error(
                    "JsonKeyDefinition::data_type_to_string: "
                    "Unknown data_type provided: " +
                    data_type);
        }
    }

    /**
     * Checks if the given JSON iterator is within the specified bounds.
     * The bounds are defined by the minimum and maximum values allowed for the
     * supported data types. Only specified bounds are checked. If no bounds are
     * specified, all values will be accepted. Every data type that is not of
     * any `number*` data type will always return true.
     *
     * @param json_iter The JSON iterator to check.
     * @return True if the JSON iterator is within the bounds or the data type
     * is not supported, false otherwise.
     */
    bool check_bounds(const nlohmann::json::const_iterator json_iter) const {
        for (const auto &data_type : data_types) {
            switch (data_type) {
                case number:
                case number_float:
                case number_integer:
                case number_unsigned:
                    // Check is only supported for the above data types
                    if (min_val.has_value() && *json_iter < min_val)
                        return false;

                    if (max_val.has_value() && *json_iter > max_val)
                        return false;

                    return true;
                default:
                    // Every other data type will default to true
                    break;
            }
        }

        // Every other data type will default to true
        return true;
    }

    /**
     * @brief Default constructor for JsonKeyDefinition.
     */
    JsonKeyDefinition(){};

    /**
     * @brief Constructs a JsonKeyDefinition object.
     *
     * This constructor initializes a JsonKeyDefinition object with the
     * specified parameters. Use this constructor if you want to allow multiple
     * data types.
     *
     * @param required A boolean indicating whether the key is required or not.
     * @param data_types A reference to an unordered set of data types.
     * @param min_val An optional minimum value for the key (default:
     * std::nullopt).
     * @param max_val An optional maximum value for the key (default:
     * std::nullopt).
     */
    JsonKeyDefinition(const bool required,
                      const std::unordered_set<data_type_t> &data_types,
                      std::optional<const float> min_val = std::nullopt,
                      std::optional<const float> max_val = std::nullopt) {
        this->required = required;
        this->data_types = data_types;
        this->min_val = min_val;
        this->max_val = max_val;
    }

    /**
     * @brief Constructs a JsonKeyDefinition object.
     *
     * This constructor initializes a JsonKeyDefinition object with the
     * specified parameters. Use this constructor if you just want to allow one
     * data type.
     *
     * @param required A boolean indicating whether the key is required or not.
     * @param data_type The data type of the key.
     * @param min_val An optional minimum value for the key (default:
     * std::nullopt).
     * @param max_val An optional maximum value for the key (default:
     * std::nullopt).
     */
    JsonKeyDefinition(const bool required, const data_type_t data_type,
                      std::optional<const float> min_val = std::nullopt,
                      std::optional<const float> max_val = std::nullopt) {
        this->required = required;
        this->data_types.insert(data_type);
        this->min_val = min_val;
        this->max_val = max_val;
    }
};

/**
 * @brief The CommandDefinitions class provides a set of static functions to
 * define and check command keys for different types.
 *
 * The CommandDefinitions class contains static functions to define the keys for
 * different commands and check if a JSON string matches the definition. It
 * provides a map of command keys and their corresponding definitions, as well
 * as a function to check a JSON string against the defined command keys.
 */
class CommandDefinitions {
   public:
    /**
     * @brief Parses and checks a string formatted JSON against a given
     * definition
     *
     * - Checks that no undefined keys are in the JSON
     * - Checks that all required keys exist
     * - Checks that all values have the correct type
     *
     * @note The function can only be used on shallow JSONs. Arrays or
     * encapsulations are not supported and will result in an exception!
     *
     * @param json_str String formatted json that should be parsed and checked
     * @param definition Definition of what keys shall be included and what type
     * they can have
     * @throws std::runtime_error with an error message why the check failed
     */
    static nlohmann::json parse_check_json(
        const std::string &json_str,
        const std::map<const std::string, const JsonKeyDefinition> &definition);

    /**
     * @brief Checks a JSON object against a given definition
     *
     * Only use this version if you already have a json object.
     * If you only have a string, use the other overload instead to do a proper
     * parsing.
     *
     * - Checks that no undefined keys are in the JSON
     * - Checks that all required keys exist
     * - Checks that all values have the correct type
     *
     * @note The function can only be used on shallow JSONs. Arrays or
     * encapsulations are not supported and will result in an exception!
     *
     * @param json_obj Json object that should be checked
     * @param definition Definition of what keys shall be included and what type
     * they can have
     * @throws std::runtime_error with an error message why the check failed
     */
    static nlohmann::json parse_check_json(
        const nlohmann::json &json_obj,
        const std::map<const std::string, const JsonKeyDefinition> &definition);

    /**
     * Returns a map containing the definition of waypoint command keys.
     *
     * The keys in the map are of type std::string, representing the names of
     * the command keys. The values in the map are of type JsonKeyDefinition,
     * representing the definition of each command key.
     *
     * @return A map containing the definition of waypoint command keys.
     */
    static const std::map<const std::string, const JsonKeyDefinition>
    get_waypoint_command_definition() {
        const std::map<const std::string, const JsonKeyDefinition> definition{
            {"target_coordinate_lat", {true, string}},
            {"target_coordinate_lon", {true, string}},
            {"pre_wait_time_ms", {false, number_unsigned, 0, 1 * 60 * 1000}},
            {"post_wait_time_ms", {false, number_unsigned, 0, 1 * 60 * 1000}},
            {"cruise_height_cm",
             {true, number_unsigned, MIN_CRUISE_HEIGHT_CM,
              MAX_FLIGHT_HEIGHT_CM}},
            {"target_height_cm",
             {true, number_unsigned, 0, MAX_FLIGHT_HEIGHT_CM}},
            {"horizontal_speed_mps",
             {true, number, 0.0, MAX_HORIZONTAL_SPEED_MPS}},
            {"vertical_speed_mps",
             {true, number, 0.0, MAX_VERTICAL_SPEED_MPS}}};

        return definition;
    }
};
}  // namespace common_lib
