#!/usr/bin/env perl

if (@ARGV < 3) {
    print STDERR "usage: $0 <datasetdirectory> <samples> <prefix>\n";
    print STDERR "datasetdirectory   must contain images!\n";
    exit;
}

my ($dset,$samples, $prefix) = @ARGV;

my @files = <$dset/*.txt>;

open(my $fo,">$prefix.log");
print $fo "cmd: @ARGV\n";
close $fo;

my %map = ();
my $min = -1;
my $max = -1;
my %fileEx = ();
my $nonex=0;
foreach my $f (@files) {
    my $n = $f;
    $n =~ s/^$dset\///;
    $n =~ s/\.txt//g;
    print "$f -> $n!\n";
    if (-e $f) {
        $fileEx{$f} = 1;
    } else {
        $fileEx{$f} = 0;
        $nonex++;
    }
}
print "Nonexisting files: $nonex\n";
if ($nonex > 0) {
    die;
}

%result =();
my $all = @files*@files;
my $idx = 0;
foreach my $f1 (@files) {
    foreach my $f2 (@files) {
        if ($fileEx{$f1}==1 and $fileEx{$f2} == 1) {
            if (! (defined $result{$f1}{$f2})) {
                my $cmd = "./bin/tester 2 $f1 $f2 $samples";
                print "$cmd\n";
                my @a = `$cmd`;
                chomp @a;
                my $res = $a[0];
                $result{$f1}{$f2} = "$res";
                $result{$f2}{$f1} = "$res";
                print "Result=$res\n";
                print STDERR "Finished: " . (100.0*$idx/$all) . " % \n";
            } else {
                print "Pair $f1 $2 has been computed!\n";
            }
        }
        $idx++;
    }
    printFullResult2();
}

#-----------------------------------------------------

sub printFullResult2() {
    open(my $fo,">$prefix.fullresults");
    foreach my $k1 (sort (keys %result)) {
        foreach my $k2 (sort (keys %{ $result{$k1}})) {
            my $f1 = $k1;
            $f1 =~ s/^$dset\///;
            $f1 =~ s/\.jpg//g;
            my $f2 = $k2;
            $f2 =~ s/^$dset\///;
            $f2 =~ s/\.jpg//g;
#            print "k1,k2=$k1 $k2 , $f1, $f2\n";
            print $fo "$f1 $f2 $result{$k1}{$k2}\n";
        }
    }
    close $fo;
}



