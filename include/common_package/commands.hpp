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
/// Maximum allowed flight height in cm
constexpr uint16_t MAX_FLIGHT_HEIGHT_CM = 120 /* [m] */ * 100;
/// Minimum required cruise height in cm
constexpr uint16_t MIN_CRUISE_HEIGHT_CM = 5 /* [m] */ * 100;
/// Maximum allowed horizontal speed in m/s
constexpr float MAX_HORIZONTAL_SPEED_MPS = 12.0 /* [m/s] */;
/// Maximum allowed vertical speed in m/s
constexpr float MAX_VERTICAL_SPEED_MPS = 3.0 /* [m/s] */;

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
    array,
    object,
} data_type_t;

struct JsonKeyDefinition {
    /// If true, the value needs to be provided
    bool required;
    /// Unsorted set of accepted data types for the specific key
    std::unordered_set<data_type_t> data_types;
    /// Min value the value should have (only works with numbers)
    std::optional<float> min_val;
    /// Max value the value should have (only works with numbers)
    std::optional<float> max_val;

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
            case string:
                return "string";
            case array:
                return "array";
            case object:
                return "object";
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
     * @note Also returns true if the json_iter has a type different from
     * `number*`
     *
     * @param json_iter The JSON iterator to check.
     * @return True if the JSON iterator is within the bounds or the data type
     * is not supported, false otherwise.
     */
    bool check_bounds(const nlohmann::json::const_iterator json_iter) const {
        JsonKeyDefinition jsk_type_check(false, number);

        for (const auto &data_type : data_types) {
            switch (data_type) {
                case number:
                case number_float:
                case number_integer:
                case number_unsigned:
                    // Check is only supported for the above data types and
                    // json_iter must be of one of those types, too
                    if (min_val.has_value() &&
                        jsk_type_check.type_check(json_iter) &&
                        *json_iter < min_val)
                        return false;

                    if (max_val.has_value() &&
                        jsk_type_check.type_check(json_iter) &&
                        *json_iter > max_val)
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
     * Checks if the given JSON iterator matches any of the allowed data types.
     *
     * @param json_iter The JSON iterator to be checked.
     * @return True if the JSON iterator matches any of the allowed data types,
     * false otherwise.
     */
    bool type_check(const nlohmann::json::const_iterator json_iter) const {
        bool type_check = false;
        for (const auto &data_type :
             data_types)  // Loop through all the allowed data types
        {
            switch (data_type) {
                case null:
                    if (json_iter->is_null()) type_check = true;
                    break;
                case boolean:
                    if (json_iter->is_boolean()) type_check = true;
                    break;
                case number:
                    if (json_iter->is_number()) type_check = true;
                    break;
                case number_float:
                    if (json_iter->is_number_float()) type_check = true;
                    break;
                case number_integer:
                    if (json_iter->is_number_integer()) type_check = true;
                    break;
                case number_unsigned:
                    if (json_iter->is_number_unsigned()) type_check = true;
                    break;
                case string:
                    if (json_iter->is_string()) type_check = true;
                    break;
                case array:
                    if (json_iter->is_array()) type_check = true;
                    break;
                case object:
                    if (json_iter->is_object()) type_check = true;
                    break;
                default:
                    throw std::runtime_error(
                        "JsonKeyDefinition::type_check: "
                        "Unknown data_type provided: " +
                        std::to_string(data_type));
            }

            if (type_check) break;
        }

        return type_check;
    }

    /**
     * @brief Default constructor for JsonKeyDefinition.
     */
    JsonKeyDefinition() = default;

    /**
     * @brief Constructs a JsonKeyDefinition object.
     *
     * This constructor initializes a JsonKeyDefinition object with the
     * specified parameters. Use this constructor if you want to allow multiple
     * data types.
     *
     * @note If max_val < min_val, the two values will be exchanged
     *
     * @param required A boolean indicating whether the key is required or not.
     * @param data_types A reference to an unordered set of data types.
     * @param min_val An optional minimum value for the key (default:
     * std::nullopt).
     * @param max_val An optional maximum value for the key (default:
     * std::nullopt).
     *
     * @throw std::runtime_error if data_types is empty
     */
    JsonKeyDefinition(const bool required,
                      const std::unordered_set<data_type_t> &data_types,
                      std::optional<const float> min_val = std::nullopt,
                      std::optional<const float> max_val = std::nullopt) {
        this->required = required;

        if (data_types.size() <= 0)
            throw std::runtime_error(
                "JsonKeyDefinition::JsonKeyDefinition: data_types is empty");

        this->data_types = data_types;

        if ((min_val.has_value() && max_val.has_value()) &&
            (min_val > max_val)) {
            // switch min_val and max_val
            this->min_val = max_val;
            this->max_val = min_val;
        } else {
            this->min_val = min_val;
            this->max_val = max_val;
        }
    }

    /**
     * @brief Constructs a JsonKeyDefinition object.
     *
     * This constructor initializes a JsonKeyDefinition object with the
     * specified parameters. Use this constructor if you just want to allow one
     * data type.
     *
     * @note If max_val < min_val, the two values will be exchanged
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
                      std::optional<const float> max_val = std::nullopt)
        : JsonKeyDefinition(required,
                            std::unordered_set<data_type_t>{data_type}, min_val,
                            max_val) {}

    /**
     * @brief Overloaded equality operator for comparing two JsonKeyDefinition
     * objects.
     *
     * This operator compares the current JsonKeyDefinition object with another
     * object of the same type. It checks if all the member variables of both
     * objects are equal.
     *
     * @param other The JsonKeyDefinition object to compare with.
     * @return true if the objects are equal, false otherwise.
     */
    constexpr bool operator==(const JsonKeyDefinition &other) const {
        return this->required == other.required &&
               this->data_types == other.data_types &&
               this->min_val == other.min_val && this->max_val == other.max_val;
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
     * - Checks that all values have the correct type and are withing bounds (if
     * applicable)
     *
     * @note The function can only be fully used on shallow JSONs. Arrays or
     * encapsulations contents' will not be checked! Call this function several
     * times with the different parts to get that behavior.
     *
     * @param json_str String formatted json that should be parsed and checked
     * @param definition Definition of what keys shall be included and what type
     * they can have
     * @throws std::runtime_error with an error message why the check failed
     */
    static nlohmann::json parse_check_json_str(
        const std::string &json_str,
        const std::map<const std::string, const JsonKeyDefinition> &definition);

    /**
     * @brief Checks a JSON object against a given definition
     *
     * Only use this version if you already have a json object.
     * If you only have a string, use the `parse_check_json_str` function
     * instead to do a proper parsing.
     *
     * - Checks that no undefined keys are in the JSON
     * - Checks that all required keys exist
     * - Checks that all values have the correct type and are withing bounds (if
     * applicable)
     *
     * @note The function can only be fully used on shallow JSONs. Arrays or
     * encapsulations contents' will not be checked! Call this function several
     * times with the different parts to get that behavior.
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
            {"target_coordinate_lat", {true, number_float}},
            {"target_coordinate_lon", {true, number_float}},
            {"pre_wait_time_ms",
             {false, number_unsigned, 0, 1 /* min */ * 60 * 1000}},
            {"post_wait_time_ms",
             {false, number_unsigned, 0, 1 /* min */ * 60 * 1000}},
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

    /**
     * Returns the command definition for the detect marker command.
     *
     * The command definition is a map that specifies the expected keys and
     * their corresponding value definitions in a JSON payload for the detect
     * marker command.
     *
     * @return The command definition for the detect marker command.
     */
    static const std::map<const std::string, const JsonKeyDefinition>
    get_detect_marker_command_definition() {
        const std::map<const std::string, const JsonKeyDefinition> definition{
            {"timeout_ms",
             {true, number_unsigned, 0, 3 /* min */ * 60 * 1000}}};

        return definition;
    }

    /**
     * Retrieves the definition for a given command type.
     *
     * @param type The type of the command.
     * @return The definition of the command as a map of string keys to
     * JsonKeyDefinition values.
     * @throws std::runtime_error if the type is unknown.
     */
    static const std::map<const std::string, const JsonKeyDefinition>
    get_definition(const std::string &type) {
        if (type == "waypoint")
            return get_waypoint_command_definition();
        else if (type == "detect_marker")
            return get_detect_marker_command_definition();
        else if (type == "end_mission")
            return {};
        else
            throw std::runtime_error(
                "CommandDefinitions::get_definition: Unknown type: " + type);
    }
};
}  // namespace common_lib
