#include "common_package/commands.hpp"

using namespace common_lib;

/**
 * @brief Parses and checks a string formatted JSON against a given definition
 *
 * - Checks that no undefined keys are in the JSON
 * - Checks that all required keys exist
 * - Checks that all values have the correct type
 *
 * @note The function can only be used on shallow JSONs. Arrays or encapsulations are not supported and will result in an exception!
 *
 * @param json_str String formatted json that should be parsed and checked
 * @param definition Definition of what keys shall be included and what type they can have
 * @throws std::runtime_error with an error message why the check failed
 */
nlohmann::json CommandDefinitions::parse_check_json(const std::string &json_str, const std::map<std::string, JsonKeyDefinition> definition)
{
    nlohmann::json candidate; // The candidate that will be checked

    // Parse string
    try
    {
        candidate = nlohmann::json::parse(json_str);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        throw std::runtime_error("CommandDefinitions::parse_check_json: Failed to parse JSON: " + std::string(e.what()));
    }

    return CommandDefinitions::parse_check_json(candidate, definition);
}

/**
 * @brief Checks a JSON object against a given definition
 *
 * Only use this version if you already have a json object.
 * If you only have a string, use the other overload instead to do a proper parsing.
 *
 * - Checks that no undefined keys are in the JSON
 * - Checks that all required keys exist
 * - Checks that all values have the correct type
 *
 * @note The function can only be used on shallow JSONs. Arrays or encapsulations are not supported and will result in an exception!
 *
 * @param json_obj Json object that should be checked
 * @param definition Definition of what keys shall be included and what type they can have
 * @throws std::runtime_error with an error message why the check failed
 */
nlohmann::json CommandDefinitions::parse_check_json(const nlohmann::json &json_obj, const std::map<std::string, JsonKeyDefinition> definition)
{
    // Check that all keys are allowed
    for (const auto &[key, val] : json_obj.items())
    {
        const auto search = definition.find(key);

        if (search == definition.end())
            throw std::runtime_error("CommandDefinitions::parse_check_json: Unknown key found: " + key);
    }

    // Check keys and value types
    for (const auto &[key, json_definition] : definition)
    {
        const auto search = json_obj.find(key); // Current value that will be checked

        if (search == json_obj.end())
        {
            if (json_definition.required) // Check that key exists if required
                throw std::runtime_error("CommandDefinitions::parse_check_json: Missing required key: " + key);
            else // If key is not required and doesn't exist, continue with next key
                continue;
        }

        // Check if value is in bound (only supported for numbers)
        if (!json_definition.check_bounds(search))
        {
            std::string error_msg = "CommandDefinitions::parse_check_json: [Key: '" + key + "'] Value '" + search->dump() + "' is out of bounds: ";

            if (json_definition.min_val.has_value() && json_definition.max_val.has_value()) // Message when min and max values are defined
                error_msg += "[" + std::to_string(json_definition.min_val.value()) + "; " + std::to_string(json_definition.max_val.value()) + "]";
            else if (json_definition.min_val.has_value()) // Message when only the min value is defined
                error_msg += "> " + std::to_string(json_definition.min_val.value());
            else if (json_definition.max_val.has_value()) // Message when only the max value is defined
                error_msg += "< " + std::to_string(json_definition.max_val.value());

            throw std::runtime_error(error_msg);
        }

        // Check that type of value matches the definition
        bool type_check = false;
        for (const auto &data_type : json_definition.data_types) // Loop through all the allowed data types
        {
            switch (data_type)
            {
            case null:
                if (search->is_null())
                    type_check = true;
                break;
            case boolean:
                if (search->is_boolean())
                    type_check = true;
                break;
            case number:
                if (search->is_number())
                    type_check = true;
                break;
            case number_float:
                if (search->is_number_float())
                    type_check = true;
                break;
            case number_integer:
                if (search->is_number_integer())
                    type_check = true;
                break;
            case number_unsigned:
                if (search->is_number_unsigned())
                    type_check = true;
                break;
            default:
                throw std::runtime_error("CommandDefinitions::parse_check_json: Unknown data_type provided: " + data_type);
            }

            if (type_check)
                break;
        }

        if (!type_check) // Type check failed
        {
            // Trying to be as helpful as possible
            std::string allowed_types = "";

            bool first_loop = true;
            for (const auto &data_type : json_definition.data_types) // Loop through all allowed data types and add them to the `allowed_types` string for the error message
            {
                if (first_loop)
                    first_loop = false;
                else
                    allowed_types += ", ";

                allowed_types += JsonKeyDefinition::data_type_to_string(data_type);
            }

            std::string error_msg = "CommandDefinitions::parse_check_json: [Key: '" + key + "'] Value '" + search->dump() + "' is of the wrong type. Allowed types: " + allowed_types;

            throw std::runtime_error(error_msg);
        }
    }

    // Json object successfully passed all checks
    return json_obj;
}
