<?php

$shm_id = shmop_open(100, 'w', 0, 0);
if (empty($shm_id))
{
    echo "Failed to open shared memory.\n";
    die();
}

if (isset($_GET['freq']))
{
    $freq = (int) $_GET['freq'];
    if ($freq < 70000 || $freq > 6000000)
	die();
    shmop_write($shm_id, pack('L', $freq), 8);

    usleep(100000);
}




//echo "\nShared memory size: ".shmop_size($shm_id)." bytes\n";


// dBm
$db = shmop_read($shm_id, 0, 4);
$db_u  = unpack("V", $db);


// counter
$ctr = shmop_read($shm_id, 4, 4);
$counter_u = unpack("V", $ctr);

// freq 
$freq = shmop_read($shm_id, 8, 4);
$freq_u = unpack("V", $freq);


// gain
$gain = shmop_read($shm_id, 12, 4);
$gain_u = unpack("V", $gain);


if (!isset($_GET['json']))
{
    echo "<PRE>";
    echo "Counter:  ". ($counter_u[1])." \n";
    echo "Freq:  ". ($freq_u[1]/1000)." MHz \n";
    echo "Power:  ". ($db_u[1]/1000)." dB\n";
    echo "Gain:  ". ($gain_u[1])." db \n";
} else
{
    $d['ctr']   = $counter_u[1];
    $d['freq']  = $freq_u[1];
    $d['power'] = $db_u[1]/1000;
    $d['gain']  = $gain_u[1];
    echo json_encode($d);
}



?>
