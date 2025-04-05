import pandas as pd
import os

def convert_to_yolo_format(csv_file, output_dir):
    """
    Convert CSV annotations to YOLO format.
    """
    # Read CSV file
    df = pd.read_csv(csv_file)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Group by filename to handle multiple objects per image
    for filename, group in df.groupby('filename'):
        # Create txt filename
        txt_filename = os.path.splitext(filename)[0] + '.txt'
        txt_path = os.path.join(output_dir, txt_filename)
        
        with open(txt_path, 'w') as f:
            for _, row in group.iterrows():
                # Get image dimensions
                img_width = row['width']
                img_height = row['height']
                
                # Convert bbox to YOLO format
                x_min = row['xmin']
                y_min = row['ymin']
                x_max = row['xmax']
                y_max = row['ymax']
                
                # Calculate center points and width/height
                x_center = ((x_min + x_max) / 2) / img_width
                y_center = ((y_min + y_max) / 2) / img_height
                width = (x_max - x_min) / img_width
                height = (y_max - y_min) / img_height
                
                # Class ID (assuming 'bulls eye' is class 0)
                class_id = 0
                
                # Write to file
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

# Usage example
convert_to_yolo_format('bulls-eye-target-images-scraped-from-google/train_labels.csv', 'labels/train')
convert_to_yolo_format('bulls-eye-target-images-scraped-from-google/validation_labels.csv', 'labels/valid')

print("Conversion completed! Please ensure your images are placed in:")
print("- labels/train/")
print("- images/train/")