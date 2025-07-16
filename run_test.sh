TEST_NUM=$1 
INPUT_FILE="tests/test${TEST_NUM}.txt"
EXPECTED_FILE="tests/expected${TEST_NUM}.txt"
OUTPUT_FILE="output${TEST_NUM}.txt"

if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: Input file $INPUT_FILE not found!"
    exit 1
fi

if [ ! -f "$EXPECTED_FILE" ]; then
    echo "Error: Expected file $EXPECTED_FILE not found!"
    exit 1
fi

./main < "$INPUT_FILE" > "$OUTPUT_FILE"

if diff -q -w "$OUTPUT_FILE" "$EXPECTED_FILE" > /dev/null; then
    echo "Test $TEST_NUM passed."
    exit 0
else
    echo "Test $TEST_NUM failed. Differences:"
    diff "$OUTPUT_FILE" "$EXPECTED_FILE"
    exit 1
fi