import os

def remove_xml_files(directory_path):
    # Count removed files
    removed_count = 0
    
    # Walk through directory
    for filename in os.listdir(directory_path):
        if filename.endswith('.xml'):
            file_path = os.path.join(directory_path, filename)
            try:
                os.remove(file_path)
                removed_count += 1
                print(f"Removed: {filename}")
            except Exception as e:
                print(f"Error removing {filename}: {e}")
    
    print(f"\nTotal XML files removed: {removed_count}")

# Example usage
directory = "/home/luiz/UAS_SD/bulls-eye-target-images-scraped-from-google/validation/"  # Replace with your directory path
remove_xml_files(directory)