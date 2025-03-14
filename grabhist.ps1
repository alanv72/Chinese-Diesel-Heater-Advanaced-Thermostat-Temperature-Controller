$baseUrl = "http://rv-dheat.local/history_"
$extension = ".json"
$numbers = 0..7

foreach ($number in $numbers) {
    $url = $baseUrl + $number + $extension
    $destination = "history_$number.json" # Update with your desired save path
    Invoke-WebRequest -Uri $url -OutFile $destination
}
