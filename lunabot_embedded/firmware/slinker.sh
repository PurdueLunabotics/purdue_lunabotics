#!/bin/bash

if [ $# != 2 ]; then
	echo "Usage: $0 <source folder> <destination folder>"
	exit 1
fi

SOURCE_DIR="$1"
TARGET_DIR="$2"

if [ ! -d "$SOURCE_DIR" ]; then
	echo "Error: source dir invalid"
	exit 1
fi

if [ ! -d "$TARGET_DIR" ]; then
	echo "Error: target dir invalid"
	exit 1
fi

for file in "$SOURCE_DIR"/*; do
	if [[ -f "$file" && ! -d "$file" ]]; then
		filename=$(basename "$file")

		ln -sf "$(realpath $file)" "$TARGET_DIR/$filename"

		echo "Linked $filename -> $TARGET_DIR"
	fi
done
