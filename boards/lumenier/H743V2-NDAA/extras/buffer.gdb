define dump_console_buffer
  set $buf = &g_console_buffer._buffer[0]
  set $head = g_console_buffer._head
  set $tail = g_console_buffer._tail
  set $size = 4096
  set $count = 0

  if $head <= $tail
    set $count = $tail - $head
  else
    set $count = $size - ($head - $tail)
  end

  set $i = $head
  while $count > 0
    printf "%c", *((char*)$buf + $i)
    set $i = ($i + 1) % $size
    set $count = $count - 1
  end
  printf "\n"
end
