#include "common_package/commands.hpp"

using namespace common_lib;

/**
 * @brief Parses and checks a string formatted JSON against a given definition
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
nlohmann::json CommandDefinitions::parse_check_json_str(
    const std::string &json_str,
    const std::map<const std::string, const JsonKeyDefinition> &definition) {
    nlohmann::json candidate;  // The candidate that will be checked

    // Parse string
    try {
        candidate = nlohmann::json::parse(json_str);
    } catch (const nlohmann::json::parse_error &e) {
        throw std::runtime_error(
            "CommandDefinitions::parse_check_json_str: Failed to parse JSON: " +
            std::string(e.what()));
    }

    return CommandDefinitions::parse_check_json(candidate, definition);
}

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
nlohmann::json CommandDefinitions::parse_check_json(
    const nlohmann::json &json_obj,
    const std::map<const std::string, const JsonKeyDefinition> &definition) {
    // Check that all keys are allowed
    {
        bool unknown_key_found = false;  // If true, unknown keys were found and
                                         // an exception will be thrown
        std::string unknown_keys =
            "";  // String with all unknown keys that were found

        for (const auto &[key, val] : json_obj.items()) {
            // Try to find key in definition
            const auto search = definition.find(key);

            // Check if key is unknown
            if (search == definition.end()) {
                if (unknown_key_found) unknown_keys += ", ";

                unknown_key_found = true;
                unknown_keys += key;
            }
        }

        // Check if at least one unknown key was found
        if (unknown_key_found) {
            throw std::runtime_error(
                "CommandDefinitions::parse_check_json: Unknown key(s) found: " +
                unknown_keys);
        }
    }

    // Check keys and value types
    for (const auto &[key, json_definition] : definition) {
        const auto search =
            json_obj.find(key);  // Current value that will be checked

        if (search == json_obj.end()) {
            if (json_definition.required)  // Check that key exists if required
                throw std::runtime_error(
                    "CommandDefinitions::parse_check_json: Missing required "
                    "key: " +
                    key);
            else  // If key is not required and doesn't exist, continue with
                  // next key
                continue;
        }

        // Check if value is in bound (only supported for numbers)
        if (!json_definition.check_bounds(search)) {
            std::string error_msg =
                "CommandDefinitions::parse_check_json: [Key: '" + key +
                "'] Value '" + search->dump() + "' is out of bounds: ";

            if (json_definition.min_val.has_value() &&
                json_definition.max_val.has_value())  // Message when min and
                                                      // max values are defined
                error_msg +=
                    "[" + std::to_string(json_definition.min_val.value()) +
                    "; " + std::to_string(json_definition.max_val.value()) +
                    "]";
            else if (json_definition.min_val
                         .has_value())  // Message when only the min value is
                                        // defined
                error_msg +=
                    "> " + std::to_string(json_definition.min_val.value());
            else if (json_definition.max_val
                         .has_value())  // Message when only the max value is
                                        // defined
                error_msg +=
                    "< " + std::to_string(json_definition.max_val.value());

            throw std::runtime_error(error_msg);
        }

        // Check that type of value matches the definition
        bool type_check = json_definition.type_check(search);

        if (!type_check)  // Type check failed
        {
            // Trying to be as helpful as possible
            std::string allowed_types = "";

            bool first_loop = true;
            for (const auto &data_type :
                 json_definition
                     .data_types)  // Loop through all allowed data types
                                   // and add them to the `allowed_types`
                                   // string for the error message
            {
                if (first_loop)
                    first_loop = false;
                else
                    allowed_types += ", ";

                allowed_types +=
                    JsonKeyDefinition::data_type_to_string(data_type);
            }

            std::string error_msg =
                "CommandDefinitions::parse_check_json: [Key: '" + key +
                "'] Value '" + search->dump(4) +
                "' is of the wrong type. Allowed type(s): " + allowed_types;

            throw std::runtime_error(error_msg);
        }
    }

    // Json object successfully passed all checks
    return json_obj;
}
