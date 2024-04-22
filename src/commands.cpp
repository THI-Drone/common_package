#include "common_package/commands.hpp"

using namespace common_lib;

/**
 * @brief Parses and checks a string formatted JSON against a given definition
 *
 * - Checks that no undefined keys are in the JSON
 * - Checks that all required keys exist
 * - Checks that all values have the correct type
 * - Checks the value is in the optional bounds (only supported for numbers)
 *
 * @note The function can only be used on shallow JSONs. Arrays or encapsulations are not supported and will result in an exception!
 *
 * @param json_str String formatted json that should be parsed and checked
 * @param definition Definition of what keys shall be included and what type they can have
 * @throws std::runtime_error with an error message why the check failed
 */
nlohmann::json CommandDefinitions::parse_check_json(const std::string &json_str, const std::map<std::string, JsonKeyDefinition> definition)
{
    nlohmann::json candidate; // The candidate that will be tested

    // Parse string
    try
    {
        candidate = nlohmann::json::parse(json_str);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        throw std::runtime_error("CommonNode::parse_check_json: Failed to parse JSON: " + std::string(e.what()));
    }

    // Check that all keys are allowed
    for (const auto &[key, val] : candidate.items())
    {
        const auto search = candidate.find(key);

        if (search == candidate.end())
            throw std::runtime_error("CommonNode::parse_check_json: Unknown key found: " + key);
    }

    // Check keys and value types
    for (const auto &[key, json_definition] : definition)
    {
        const auto search = candidate.find(key); // Current value that will be checked

        if (search == candidate.end())
        {
            if (json_definition.required) // Check that key exists if required
                throw std::runtime_error("CommonNode::parse_check_json: Missing required key: " + key);
            else // If key is not required and doesn't exist, continue with next key
                continue;
        }

        // Check type and bounds (only for numbers)
        /**
         * @brief Checks if the value pointed to by `search` is within the specified bounds.
         *
         * @return true if the value is within the bounds, false otherwise.
         */
        auto check_bounds = [search, json_definition]() -> bool
        {
            if (json_definition.min_val.has_value() && *search < json_definition.min_val)
                return false;

            if (json_definition.max_val.has_value() && *search > json_definition.max_val)
                return false;

            return true;
        };

        bool type_check = false;
        for (const auto &data_type : json_definition.data_types)
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
                if (search->is_number() && check_bounds())
                    type_check = true;
                break;
            case number_float:
                if (search->is_number_float() && check_bounds())
                    type_check = true;
                break;
            case number_integer:
                if (search->is_number_integer() && check_bounds())
                    type_check = true;
                break;
            case number_unsigned:
                if (search->is_number_unsigned() && check_bounds())
                    type_check = true;
                break;
            default:
                throw std::runtime_error("CommonNode::parse_check_json: Unknown data_type provided: " + data_type);
            }

            if (type_check)
                break;
        }

        if (!type_check) // Type check failed
        {
            // Trying to be as helpful as possible
            std::string allowed_types = "";

            bool first_loop = true;
            for (const auto &data_type : json_definition.data_types)
            {
                if (first_loop)
                    first_loop = false;
                else
                    allowed_types += ", ";

                allowed_types += JsonKeyDefinition::data_type_to_string(data_type);
            }

            std::string error_msg = "CommonNode::parse_check_json: [Key: '" + key + "'] Value '" + search->dump() + "' is of the wrong type";
            if (json_definition.min_val.has_value() || json_definition.max_val.has_value())
            {
                error_msg += " or out of bounds: ";
                if (json_definition.min_val.has_value() && json_definition.max_val.has_value())
                    error_msg += "[" + std::to_string(json_definition.min_val.value()) + "; " + std::to_string(json_definition.max_val.value()) + "]";
                else if (json_definition.min_val.has_value())
                    error_msg += "> " + std::to_string(json_definition.min_val.value());
                else if (json_definition.max_val.has_value())
                    error_msg += "< " + std::to_string(json_definition.max_val.value());
            }
            error_msg += ". Allowed types: " + allowed_types;

            throw std::runtime_error(error_msg);
        }
    }

    // Candidate successfully passed all checks
    return candidate;
}
