#!/usr/bin/ruby

require 'rubygems'
require 'optparse'
require 'pp'
require 'parallel'
require 'fileutils'

#alias :puts_original :puts
#
#def puts (*args)
#  puts_original(args)
#  puts_original(args)
#end

$options = {}

optparse = OptionParser.new do|opts|
  opts.banner = "Usage: run.rb [options] <path>"

  $options[:base_path] = "."

  $options[:resize_to] = 1200
  opts.on('-r', '--resize-to int', "") do |param|
    $options[:resize_to] = param.to_i
  end

  $options[:match_size] = 200
  opts.on('-m', '--match-size int', "") do |param|
    $options[:match_size] = param.to_i
  end

  $options[:start_with] = "resize"
  opts.on('', '--start-with ', "values: \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\"") do |param|
    $options[:start_with] = param
  end

  $options[:end_with]  = "pmvs"
  opts.on('', '--end-with ', "values: \"resize\", \"getKeypoints\", \"match\", \"bundler\", \"cmvs\", \"pmvs\"") do |param|
    $options[:end_with] = param
  end

  $options[:force_ccd] = false
  opts.on('', '--force-ccd float', "") do |param|
    $options[:force_ccd] = param.to_f
  end


  $options[:force_focal] = false
  opts.on('', '--force-focal float', "") do |param|
    $options[:force_focal] = param.to_f
  end

  $options[:cmvs_max_images] = 100
  opts.on('', '--cmvs-max-images int', "") do |param|
    $options[:cmvs_max_images] = param.to_i
  end

  $options[:matcher_ratio] = 0.6
  opts.on('', '--matcher-ratio float', "") do |param|
    $options[:matcher_ratio] = param.to_f
  end

  $options[:matcher_threshold] = 2.0
  opts.on('', '--matcher-threshold float', "") do |param|
    $options[:matcher_threshold] = param.to_f
  end

  $options[:pmvs_min_image_num] = 3
  opts.on('', '--pmvs-minImageNum n', "") do |param|
    $options[:pmvs_min_image_num] = param.to_i
  end
  $options[:pmvs_wsize] = 7
  opts.on('', '--pmvs-wsize int', "") do |param|
    $options[:pmvs_wsize] = param.to_i
  end
  $options[:pmvs_threshold] = 0.7
  opts.on('', '--pmvs-threshold float', "") do |param|
    $options[:pmvs_threshold] = param.to_f
  end
  $options[:pmvs_csize] = 2
  opts.on('', '--pmvs-csize int', "") do |param|
    $options[:pmvs_csize] = param.to_i
  end
  $options[:pmvs_level] = 1
  opts.on('', '--pmvs-level int', "") do |param|
    $options[:pmvs_level] = param.to_i
  end

  opts.on( '-h', '--help', 'Display this screen' ) do
    puts opts
    exit
  end
end

begin
  optparse.parse!

  $options[:path_base] = ARGV[0] if ARGV.length == 1
  $options[:path_base] = File.expand_path($options[:path_base])

  $options[:path_bin]  = File.expand_path(File.dirname(__FILE__)) + "/bin"

  require "#{File.expand_path(File.dirname(__FILE__))}/ccd_defs.rb"

 begin
    puts "  Configuration:"
    puts "             bin_path = #{$options[:path_bin]}"
    puts "            base_path = #{$options[:path_base]}"
    puts "   "
    puts "           start_with = #{$options[:start_with]}"
    puts "             end_with = #{$options[:end_with]}"
    puts "   "
    puts "            resize_to = #{$options[:resize_to]}"
    puts "   "
    puts "           match_size = #{$options[:match_size]}"
    puts "        matcher_ratio = #{$options[:matcher_ratio]}"
    puts "    matcher_threshold = #{$options[:matcher_threshold]}"
    puts "   "
    puts "      cmvs_max_images = #{$options[:cmvs_max_images]}"
    puts "   "
    puts "       pmvs_threshold = #{$options[:pmvs_threshold]}"
    puts "           pmvs_csize = #{$options[:pmvs_csize]}"
    puts "           pmvs_level = #{$options[:pmvs_level]}"
    puts "   pmvs_min_image_num = #{$options[:pmvs_min_image_num]}"
    puts "           pmvs_wsize = #{$options[:pmvs_wsize]}"
    puts "   "
  end

  file_objects = []

  source_files = `ls -1 #{$options[:path_base]} | egrep "\.[jJ]{1}[pP]{1}[eE]{0,1}[gG]{1}"`.split("\n")
  file_objects = Parallel.map(source_files) { |source_file|
    file_object = Hash.new

    file_object[:file_name]     = "#{source_file}"
    file_object[:file_basename] = "#{source_file}".sub(/\.[^\.]+$/, "")
    file_object[:path_src]      = "#{$options[:path_base]}/#{source_file}"

    jhead_text = `jhead #{file_object[:path_src]}`
    file_object[:jhead] = Hash.new

    jhead_text.split("\n").each { |jhead_line|
      jhead_parts = jhead_line.split(/\ +:\ /)

      file_object[:jhead][jhead_parts[0].to_sym] = jhead_parts[1] if jhead_parts.length == 2
    }

    file_object[:model_id]  = "#{file_object[:jhead][:'Camera make']} #{file_object[:jhead][:'Camera model']}"

    file_object[:width],file_object[:height]     = [file_object[:jhead][:Resolution].split(" x ")[0].to_i, file_object[:jhead][:Resolution].split(" x ")[1].to_i] if file_object[:jhead][:Resolution]


    if file_object[:jhead][:'CCD width']
      file_object[:ccd]       = file_object[:jhead][:'CCD width'][/([\.0-9]+)/].to_f
    end

    if file_object[:jhead][:'Focal length']
      file_object[:focal]     = file_object[:jhead][:'Focal length'][/([\.0-9]+)/].to_f
    end

    file_object[:ccd]       = $ccd_widths[file_object[:model_id].to_sym] unless file_object[:ccd]

    file_object[:focal]     = $options[:force_focal] if $options[:force_focal]
    file_object[:ccd]       = $options[:force_ccd]   if $options[:force_ccd]

    if file_object[:focal] && file_object[:ccd] && file_object[:width] && file_object[:height]
      file_object[:focal_px]  = (file_object[:focal] / file_object[:ccd]) * [[file_object[:width], file_object[:height]].max.to_f, $options[:resize_to].to_f].min
      puts "#{file_object[:path_src]} – using image with ccd width: #{file_object[:ccd]} and focal length: #{file_object[:focal]}"
      file_object
    else
      if !file_object[:ccd]
        puts "no ccd width found for #{file_object[:model_id]}"
      end
    end
  }

  file_objects = file_objects.select {|file_object| file_object}

  puts "found #{file_objects.length} usable objects"

  def get_feature_count (bin_file_name)
    io = File.open(bin_file_name, "rb")
    feature_count = io.read(4).unpack("L")
    io.close

    feature_count.first.to_i
  end

  def match?(i, j, file_object_i, file_object_j, path)
    pairwise_match = "#{path}/#{i}-#{j}.txt"

    !File.exists?(pairwise_match)
  end
  def match (i, j, file_object_i, file_object_j, path, index)
    pairwise_match = "#{path}/.#{i}-#{j}.txt"
    done_pairwise_match = "#{path}/#{i}-#{j}.txt"

    file_object_i_key = "#{path}/../#{file_object_i[:file_basename]}.key"
    file_object_j_key = "#{path}/../#{file_object_j[:file_basename]}.key"

    feature_count_i = get_feature_count("#{path}/../#{file_object_i[:file_basename]}.key.bin")
    feature_count_j = get_feature_count("#{path}/../#{file_object_j[:file_basename]}.key.bin")

    `touch '#{pairwise_match}' && '#{$options[:path_bin]}/KeyMatch' '#{file_object_i_key}' '#{file_object_j_key}' '#{pairwise_match}' #{$options[:matcher_ratio]} #{$options[:matcher_threshold]}` unless  File.exists?(pairwise_match)
    matches =`cat '#{pairwise_match}' | wc -l`.to_i

    if matches > 0
      puts "%6d / %6d - %6.2f%% matches between #{file_object_i[:file_name]}, #{file_object_j[:file_name]}" % [$prog_start + index + 1, $prog_total, matches.to_f*100/([feature_count_i, feature_count_j].min)]
    end

    FileUtils.mv(pairwise_match, done_pairwise_match)
  end

  def get_keypoints_for_file?(file_object, path, size)
    path_matching_base_name = "#{path}/#{file_object[:file_basename]}"
    path_matching_key_bin   = "#{path_matching_base_name}.key.bin"
    path_matching_key_gz    = "#{path_matching_base_name}.key.gz"
    path_matching_jpg       = "#{path_matching_base_name}.jpg"

    !(File.exists?(path_matching_jpg) && File.exists?(path_matching_key_bin) && File.exists?(path_matching_key_gz))
  end
  def get_keypoints_for_file (file_object, path, size, index)
    path_matching_base_name = "#{path}/.#{file_object[:file_basename]}"
    path_matching_jpg       = "#{path_matching_base_name}.jpg"
    path_matching_pgm       = "#{path_matching_base_name}.pgm"
    path_matching_sift      = "#{path_matching_base_name}.key.sift"
    path_matching_key       = "#{path_matching_base_name}.key"
    path_matching_key_bin   = "#{path_matching_base_name}.key.bin"
    path_matching_key_gz    = "#{path_matching_base_name}.key.gz"

    done_path_matching_base_name  = "#{path}/#{file_object[:file_basename]}"
    done_path_matching_key_bin    = "#{done_path_matching_base_name}.key.bin"
    done_path_matching_key_gz     = "#{done_path_matching_base_name}.key.gz"
    done_path_matching_jpg        = "#{done_path_matching_base_name}.jpg"


    `convert -format jpg -resize #{size}x#{size} -quality 100 '#{file_object[:path_src]}' '#{path_matching_jpg}'`
    `convert -format pgm -resize #{size}x#{size} -quality 100 '#{file_object[:path_src]}' '#{path_matching_pgm}'`

    `'#{$options[:path_bin]}/vlsift' '#{path_matching_pgm}' -o '#{path_matching_sift}'`

    `perl '#{$options[:path_bin]}/../convert_vlsift_to_lowesift.pl' '#{path_matching_base_name}'`

    `gzip -f '#{path_matching_key}' && rm -f '#{path_matching_sift}' && rm -f '#{path_matching_pgm}'`

    feature_count = get_feature_count(path_matching_key_bin)

    puts "%6d / %6d - got #{feature_count} keypoints from #{file_object[:file_name]} @ #{size}px" % [$prog_start + index + 1, $prog_total]

    FileUtils.mv(path_matching_key_bin  , done_path_matching_key_bin  )
    FileUtils.mv(path_matching_key_gz   , done_path_matching_key_gz   )
    FileUtils.mv(path_matching_jpg      , done_path_matching_jpg      )
  end

  if file_objects.length > 0
    job_options  = Hash.new

    job_options[:path] = "#{$options[:path_base]}/__reconstruction-#{$options[:resize_to]}"

    Dir::mkdir(job_options[:path]) unless File.directory?(job_options[:path])

    ### MATCHING

      job_options[:path_matching]             = "#{$options[:path_base]}/__pre_matching-#{$options[:match_size]}"
      job_options[:path_matchinglarge]        = "#{job_options[:path]}"
      job_options[:path_matching_pairs]       = "#{$options[:path_base]}/__pre_matching-#{$options[:match_size]}/_pairs"
      job_options[:path_matchinglarge_pairs]  = "#{job_options[:path_matchinglarge]}/_pairs"

      Dir::mkdir(job_options[:path_matching])             unless File.directory?(job_options[:path_matching])
      Dir::mkdir(job_options[:path_matchinglarge])        unless File.directory?(job_options[:path_matchinglarge])
      Dir::mkdir(job_options[:path_matching_pairs])       unless File.directory?(job_options[:path_matching_pairs])
      Dir::mkdir(job_options[:path_matchinglarge_pairs])  unless File.directory?(job_options[:path_matchinglarge_pairs])



      puts "\n**\n** GETTING KEYPOINTS SMALL VERSION\n** #{Time.now}\n\n"

      file_objects_todo = file_objects.select     { |file_object| get_keypoints_for_file?(file_object, job_options[:path_matching], $options[:match_size]) }

      $prog_start = (file_objects.length - file_objects_todo.length)
      $prog_total = file_objects.length
      Parallel.each_with_index(file_objects_todo) { |file_object, index| get_keypoints_for_file(file_object, job_options[:path_matching], $options[:match_size], index) }

      puts (file_objects_todo.empty? ? "nothing to do" : "done")



      puts "\n**\n** MATCHING SMALL VERSION\n** #{Time.now}\n\n"

      match_indeces = Array.new

      (0...file_objects.length).inject(match_indeces) { |memo, i|
        (i+1...file_objects.length).inject(memo) { |memo, j|
          memo.push([i, j])
        }
      }

      match_indeces_todo = match_indeces.select     { |i,j| match?(i, j, file_objects[i], file_objects[j], job_options[:path_matching_pairs]) }

      $prog_start = (match_indeces.length - match_indeces_todo.length)
      $prog_total = match_indeces.length
      Parallel.each_with_index(match_indeces_todo)  { |(i, j), index|
        match(i, j, file_objects[i], file_objects[j], job_options[:path_matching_pairs], index)
      }

      puts (match_indeces_todo.empty? ? "nothing to do" : "done")



      puts "\n**\n** GETTING KEYPOINTS BIG VERSION\n** #{Time.now}\n\n"

      file_objects_todo = file_objects.select     { |file_object| get_keypoints_for_file?(file_object, job_options[:path_matchinglarge], $options[:resize_to]) }

      $prog_start = (file_objects.length - file_objects_todo.length)
      $prog_total = file_objects.length
      Parallel.each_with_index(file_objects_todo, :in_processes => 4)  { |file_object, index| get_keypoints_for_file(file_object, job_options[:path_matchinglarge], $options[:resize_to], index) }

      puts (file_objects_todo.empty? ? "nothing to do" : "done")



      puts "\n**\n** MATCHING BIG VERSION\n** #{Time.now}\n\n"

      matches_files = `ls -1 #{job_options[:path_matching_pairs]} | egrep "\.txt"`.split("\n").map {|e| (e.sub("\.txt", "").split("-")).map{|n| n.to_i} }
      matches_files = matches_files.select {|i,j| File.size?("#{job_options[:path_matching_pairs]}/#{i}-#{j}.txt") }

      matches_files_todo = matches_files.select    { |i, j| match?(i, j, file_objects[i], file_objects[j], job_options[:path_matchinglarge_pairs]) }

      $prog_start = (matches_files.length - matches_files_todo.length)
      $prog_total = matches_files.length
      Parallel.each_with_index(matches_files_todo) { |(i, j), index| match(i, j, file_objects[i], file_objects[j], job_options[:path_matchinglarge_pairs], index)  }

      puts (matches_files_todo.empty? ? "nothing to do" : "done")



      puts "\n**\n** RUNNING BUNDLER\n** #{Time.now}\n\n"

      job_options[:path_bundle]         = "#{job_options[:path]}/bundle"
      job_options[:path_pmvs]           = "#{job_options[:path]}/pmvs"
      job_options[:path_pmvs_txt]       = "#{job_options[:path]}/pmvs/txt"
      job_options[:path_pmvs_visualize] = "#{job_options[:path]}/pmvs/visualize"
      job_options[:path_pmvs_models]    = "#{job_options[:path]}/pmvs/models"

      job_options[:file_bundler_filelist]     = "#{job_options[:path]}/_bundler_list.txt"
      job_options[:file_bundler_options]      = "#{job_options[:path]}/_bundler_options.txt"
      job_options[:file_bundler_matches_init] = "#{job_options[:path]}/_bundler_matches.init.txt"

      Dir::mkdir(job_options[:path_bundle])         unless File.directory?(job_options[:path_bundle])
      Dir::mkdir(job_options[:path_pmvs])           unless File.directory?(job_options[:path_pmvs])
      Dir::mkdir(job_options[:path_pmvs_txt])       unless File.directory?(job_options[:path_pmvs_txt])
      Dir::mkdir(job_options[:path_pmvs_visualize]) unless File.directory?(job_options[:path_pmvs_visualize])
      Dir::mkdir(job_options[:path_pmvs_models])    unless File.directory?(job_options[:path_pmvs_models])

#      files_for_bundler = `ls -1 #{job_options[:path_matchinglarge]} | egrep "\.[jJ]{1}[pP]{1}[eE]{0,1}[gG]{1}$"`.split("\n").sort
#      files_for_bundler = file_objects.select  { |file_object| files_for_bundler.include?(file_object[:file_name]) }

      File.open(job_options[:file_bundler_filelist], 'w') do |file|
        file_objects.each { |file_object|
          file.puts "./%s 0 %0.5f" % ["#{file_object[:file_basename]}.jpg", file_object[:focal_px]]
        }
      end

      File.open(job_options[:file_bundler_options], 'w') do |file|
        file.puts "--match_table _bundler_matches.init.txt"
        file.puts "--output bundle.out"
        file.puts "--output_all bundle_"
        file.puts "--output_dir bundle"
        file.puts "--variable_focal_length"
        file.puts "--use_focal_estimate"
        file.puts "--constrain_focal"
        file.puts "--constrain_focal_weight 0.01"
        file.puts "--estimate_distortion"
        file.puts "--run_bundle"
      end

      File.open(job_options[:file_bundler_matches_init], 'w') do |file|
        matches_files = `ls -1 #{job_options[:path_matchinglarge_pairs]} | egrep "\.txt"`.split("\n").map {|e| (e.sub("\.txt", "").split("-")).map{|n| n.to_i} }
        matches_files = matches_files.sort

        matches_files.each {|i,j|
          if File.size?("#{job_options[:path_matchinglarge_pairs]}/#{i}-#{j}.txt")
            file.puts "#{i} #{j}"
            file.puts File.read("#{job_options[:path_matchinglarge_pairs]}/#{i}-#{j}.txt")
          end
        }
      end

      Dir.chdir("#{job_options[:path]}")

      system("'#{$options[:path_bin]}/bundler'         '#{job_options[:file_bundler_filelist]}' --options_file '#{job_options[:file_bundler_options]}'")

      puts "\n**\n** RUNNING BUNDLE2PMVS\n** #{Time.now}\n\n"
      system("'#{$options[:path_bin]}/Bundle2PMVS'     '#{job_options[:file_bundler_filelist]}' 'bundle/bundle.out'")

      puts "\n**\n** RUNNING RADIALUNDISTORT\n** #{Time.now}\n\n"
      system("'#{$options[:path_bin]}/RadialUndistort' '#{job_options[:file_bundler_filelist]}' 'bundle/bundle.out' 'pmvs'")

      i = 0

      file_objects.each { |file_object|
        if File.exist?("#{job_options[:path]}/pmvs/#{file_object[:file_basename]}.rd.jpg")
          nr = "%08d" % [i]

          puts "#{job_options[:path]}/pmvs/#{file_object[:file_basename]}.rd.jpg", "#{job_options[:path]}/pmvs/visualize/#{nr}.jpg"

          FileUtils.mv("#{job_options[:path]}/pmvs/#{file_object[:file_basename]}.rd.jpg", "#{job_options[:path]}/pmvs/visualize/#{nr}.jpg")
          FileUtils.mv("#{job_options[:path]}/pmvs/#{nr}.txt", "#{job_options[:path]}/pmvs/txt/#{nr}.txt")

          i += 1
        end
      }

      puts "\n**\n** RUNNING CMVS\n** #{Time.now}\n\n"
      system("'#{$options[:path_bin]}/cmvs' pmvs/ #{$options[:cmvs_max_images]} #{Parallel.processor_count}");

      puts "\n**\n** GENOPTION CMVS\n** #{Time.now}\n\n"
      system("'#{$options[:path_bin]}/genOption' pmvs/ #{$options[:pmvs_level]} #{$options[:pmvs_csize]} #{$options[:pmvs_threshold]} #{$options[:pmvs_wsize]} #{$options[:pmvs_min_image_num]} #{Parallel.processor_count}");

      puts "\n**\n** GENOPTION PMVS\n** #{Time.now}\n\n"
      system("'#{$options[:path_bin]}/pmvs2' pmvs/ option-0000");

  end
end
