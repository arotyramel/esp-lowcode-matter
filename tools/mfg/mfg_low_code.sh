#!/usr/bin/env bash

set -e

# Define color codes
RED='\033[31m'
RESET='\033[0m'

# Function to print error messages
error() {
    echo -e "${RED}Error: $1${RESET}"
}

# Function to create a status JSON file
create_status_json() {
    local product_folder="$1"
    local status="$2"
    local description="$3"
    local details="$4"
    local mac_addr="$5"
    local status_file="$product_folder/configuration/output/$mac_addr/status.json"

    echo "{\"status\": \"$status\", \"description\": \"$description\", \"details\": \"$details\"}" > "$status_file"
}

# Function to check if required environment variables are set
check_dependencies() {
    local missing_vars=()

    for var in "$@"; do
        if [ -z "${!var}" ]; then
            error "$var is not set. Please set it before running this script."
            missing_vars+=("$var")
        fi
    done

    if [ "${#missing_vars[@]}" -ne 0 ]; then
        local msg="Missing environment variables: ${missing_vars[*]}"
        local pretty_var=$(echo "$var" | tr '[:upper:]' '[:lower:]' | tr '_' '-')
        local fix="Please set them using: export $var=/path/to/$pretty_var"
        create_status_json "$product_folder" "Failure" "$msg" "$fix" "$mac_address"
        exit 1
    fi
}

# Take the path to product folder as an input
if [ $# -eq 0 ]; then
    error "Please provide the path to the product folder as an argument."
    echo "Usage: $0 <path_to_product_folder> [chip] [mac_address] [connection_type]"
    echo "  connection_type: wifi (default) or thread"
    exit 1
fi

product_folder="$1"
chip="$2"
mac_address="$3"
connection_type="${4:-wifi}"  # Default to wifi if not provided

# Validate connection_type
if [ "$connection_type" != "wifi" ] && [ "$connection_type" != "thread" ]; then
    error "Invalid connection_type: $connection_type. Must be 'wifi' or 'thread'"
    exit 1
fi

# Validate if the provided path exists and is a directory
if [ ! -d "$product_folder" ]; then
    error "The provided path '$product_folder' is not a valid directory."
    exit 1
fi

# Print the product folder path for confirmation
product_folder=$(realpath "$product_folder")
echo "Using product folder: $product_folder"
mkdir -p "$product_folder/configuration/output/$mac_address"

# checks if required environment variables exists
check_dependencies LOW_CODE_PATH ESP_MATTER_PATH ZAP_INSTALL_PATH

# Find the appropriate .zap file based on device type
zap_filename="data_model_${connection_type}.zap"
zap_file=$(find "$(realpath "$product_folder/configuration")" -name "$zap_filename" -print -quit)

# Check if a .zap file was found
if [ -z "$zap_file" ]; then
    error "No $zap_filename file found in $product_folder/configuration"
    create_status_json "$product_folder" "Failure" "No $zap_filename file found" "Check the logs for more details" "$mac_address"
    exit 1
fi

echo "Using .zap file for $connection_type device: $zap_file"
if [ "$zap_file" != "$product_folder/configuration/output/$mac_address/data_model.zap" ]; then
    cp "$zap_file" "$product_folder/configuration/output/$mac_address/data_model.zap"
fi
zap_file=$(realpath "$product_folder/configuration/output/$mac_address/data_model.zap")

# Use the found .zap file as input for the matter_data_model_serializer.py script
cd "$LOW_CODE_PATH/tools/dependencies/matter_data_model_interpreter/matter_data_model_serializer"
python3 matter_data_model_serializer.py -z "$zap_file" --chip-sdk-path "$ESP_MATTER_PATH/connectedhomeip/connectedhomeip" --no-nvs-bin

# Check if the script executed successfully
if [ $? -ne 0 ]; then
    error "Failed to execute matter_data_model_serializer.py"
    create_status_json "$product_folder" "Failure" "Failed to execute matter_data_model_serializer.py" "Check the logs for more details" "$mac_address"
    exit 1
fi

# copy matter_data_model_serializer.py generated files to output folder
cp -r serializer_output/data_model/* "$product_folder/configuration/output/$mac_address/"

echo "Successfully generated binary and JSON files from .matter file"

cp "$product_folder/configuration/output/$mac_address/data_model.bin" "$product_folder/configuration/data_model.bin"

# Change directory to mfg folder
cd "$LOW_CODE_PATH/tools/mfg"

exit_code=0
python3 mfg_gen.py --product configuration --products_path $product_folder --output_path $product_folder/configuration/output/$mac_address --local_claim --no_rainmaker --no_signature --no_ota_decryption --not_connected_device_details $chip $mac_address --no_io_validation --no_info_validation || exit_code=$?

if [ $exit_code -ne 0 ]; then
    error "mfg_gen.py execution failed"
    create_status_json "$product_folder" "Failure" "mfg_gen.py execution failed" "Check the logs for more details" "$mac_address"
    exit
fi
cd -
create_status_json "$product_folder" "Success" "Successfully generated binary and JSON files from .matter file" "Check the logs for more details" "$mac_address"
