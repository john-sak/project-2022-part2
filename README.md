# Κ23γ: Ανάπτυξη Λογισμικού για Αλγοριθμικά Προβλήματα

**Χειμερινό εξάμηνο 2022-23**
- Σακαρέλλος Ιωάννης (sdi1800167)
- Μασούρας Νικόλαος (sdi1800112)

# 2<sup>η</sup> Προγραμματιστική Εργασία

## Περιγραφή εργασίας

**Πολυγωνοποίηση σημειοσυνόλου βέλτιστης επιφάνειας με τη χρήση της βιβλιοθήκης CGAL (C++)**

Για τη συγκεκριμένη εργασία χρησιμοποιείται από προεπιλογή ο αλγόριθμος `icremental` για την παραγωγή της αρχικής πολυγωνικής γραμμής. Αυτό μπορεί να αλλάξει από το αρχείο `main/main.cpp`, στη γραμή 10. Ο αλγόριθμος `simulated annealing` στην περίπτωση του `subdivision` δεν παρουσίαζε αποτελέσματα ακόμα και μετά από αρκετή ώρα. Παρόλο που η εκτέλεση του προγράμματος δε διακοπτόταν, αργούσε πολύ να ολοκληρωθεί για τα παραδείγματα που δοκιμάστηκαν.

## Οδηγίες μεταγλώττισης

Όπως και στην πρώτη εργασία, για την παραγωγή του εκτελέσιμου αρχείου `optimal_polygon`, ακολουθούμε τα εξής βήματα:
- Μεταβαίνουμε στον φάκελο `build`
- Δίνουμε την εντολή `cmake .. -DCMAKE_BUILD_TYPE=Release`
- Δίνουμε την εντολή `make`
- Μεταβαίνουμε στον φάκελο `main` που δημιουργήθηκε
- Εκτελούμε το τελικό εκτελέσιμο αρχείο με την εντολή `./optimal_polygon` ακολουθούμενη από τα κατάλληλα ορίσματα

## Οδηγίες χρήσης

Η εκτέλεση του προγράμματος γίνεται μέσω της εντολής `$./optimal_polygon -i <input_file> -o <output_file> -algorithm <algorithm> -L <L_parameter> -max [OR -min] -threshold <threshold> -annealing <annealing>`, όπου:
- Το `<input_file>` είναι το όνομα του αρχείου που περιέχει τα δεδομένα εισόδου
- Το `<output_file>` είναι το όνομα του αρχείου στο ποίο θα γραφούν τα δεδομένα εξόδου. **Προσοχή**: Το αρχείο `<output_file>` δεν πρέπει να υπάρχει ήδη
- Το `<algorithm>` πρέπει να είναι `local_search` ή `simulated_annealing` ανάλογα με τον αλγόριθμο που πρέπει να εκτελεστεί
- Το `<L_parameter>` πρέπει να είναι η επιθυμητή - ακέραια - τιμή για την παράμετρο L.
- Το `-min` ή `-max` **(μόνο ένα από τα δύο)** πρέπει να υπάρχει για την επιλογή μεταξύ προβλήματος ελαχιστοποίησης ή μεγιστοποίησης του εμβαδού της πολυγωνικής γραμμής.
- Το `threshold` πρέπει να χρησιμοποιείται μόνο με τον αλγόριθμο `local_search`. Είναι ένας `double` αριθμός που ορίζει την ακρίβεια του προβλήματος βελτιστοποίησης.
- Το `annealing` πρέπει να χρησιμοποιείται μόνο με τον αλγόριθμο `simulated_annealing`. Πρέπει να είναι είτε `local`, είτε `global`, είτε `subdivision`, ανάλογα με την επιθυμητή παραλαγή του αλγορίθμου.

## Περιγραφή αρχείων κώδικα

### Βιβλιοθήκη `arguments` (Παρόμοια με 1<sup>η</sup> Εργασία)

Η βιβλιοθήκη `arguments` αποτελείται από τα αρχεία `arguments.cpp` και `arguments.hpp`. Υλοποιεί την κλάση που διαβάζει, επαληθεύει και αποθηκεύει τα ορίσματα που δίνει ο χρήστης κατά την εκτέλεση του προγράμματος `to_polygon`. Συγκεκριμένα, αποθηκεύει:
- Το όνομα του αρχείου με τα σημεία προς επεξεργασία (`in_file`)
- Το όνομα του αρχείου όπου θα γραφτεί η έξοδος του προγράμματος (`out_file`)
- Τον αλγόριθμο που θα χρησιμοποιηθεί (`local_search` ή `simulated_annealing`)
- Την παράμετρο `L`
- Την επιλογή μεταξύ προβλήματος ελαχιστοποίησης και μεγιστοποίησης εμβαδού
- Την ακρίβεια του αλγορίθμου `local_search`
- Την παραλαγή του αλγορίθμου `simulated_annealing`

### Βιβλιοθήκη `polyline` (Παρόμοια με 1<sup>η</sup> Εργασία)

Η βιβλιοθήκη `polyline` αποτελείται από τα αρχεία `polyline.cpp` και `polyline.hpp`. Υλοποιεί την κλάση `polyline`, μέσω της οποίας γίνεται η κατασκευή της αρχικής πολυγωνικής γραμμής, όπως στην 1<sup>η</sup> Εργασία.

### Βιβλιοθήκη `optimization`

Η βιβλιοθήκη `optimization` αποτελείται από τα αρχεία `optimization.hpp` και `optimization.cpp`. Υλοποιεί το ζητούμενο πρόβλημα βελτιστοποίησης για αυτή την Εργασία.

## Σχολιασμός αποτελεσμάτων

| File | Initialization | Algorithm | L | Min/Max | Threshold | Annealing | Init Ratio | Optimized Ratio | Time (msec) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| euro-night-0000010 | incremental | local_search | 2 | max | 0.1 | - | 0.672225 | 0.846002 | 1 |
| euro-night-0000010 | incremental | local_search | 5 | max | 0.1 | - | 0.672225 | 0.863394 | 4 |
| euro-night-0000010 | incremental | local_search | 2 | min | 0.1 | - | 0.672225 | 0.607459 | 0 |
| euro-night-0000010 | incremental | local_search | 5 | min | 0.1 | - | 0.672225 | 0.6068 | 1 |
| euro-night-0000020 | incremental | local_search | 5 | max | 0.1 | - | 0.465846 | 0.466686 | 8 |
| euro-night-0000020 | incremental | local_search | 10 | max | 0.1 | - | 0.465846 | 0.545381 | 12 |
| euro-night-0000020 | incremental | local_search | 10 | min | 0.1 | - | 0.465846 | 0.28569 | 6 |
| euro-night-0000050 | incremental | local_search | 10 | max | 0.1 | - | 0.506611 | 0.51712 | 108 |
| euro-night-0000050 | incremental | local_search | 20 | max | 0.1 | - | 0.506611 | 0.556347 | 155 |
| euro-night-0000050 | incremental | local_search | 20 | min | 0.1 | - | 0.506611 | 0.486275 | 71 |
| euro-night-0000070 | incremental | local_search | 10 | max | 0.1 | - | 0.448288 | 0.472937 | 537 |
| euro-night-0000070 | incremental | local_search | 10 | min | 0.1 | - | 0.448288 | 0.442991 | 136 |
| euro-night-0000300 | incremental | local_search | 20 | max | 0.1 | - | 0.545025 | 0.54527 | 27146 |
| euro-night-0000500 | incremental | local_search | 30 | min | 0.1 | - | 0.497843 | 0.497312 | 81504 |
| euro-night-0000010 | incremental | simulated_annealing | 10 | max | - | global | 0.672225 | 0.788115 | 6630 |
| euro-night-0000010 | incremental | simulated_annealing | 10 | max | - | local | 0.672225 | 0.788115 | 11512 |
| euro-night-0000050 | incremental | simulated_annealing | 10 | max | - | global | 0.506611 | 0.513605 | 111289 |
| euro-night-0000050 | incremental | simulated_annealing | 10 | max | - | local | 0.506611 | 0.510716 | 9561 |
| euro-night-0000500 | incremental | simulated_annealing | 50 | max | - | global | 0.497843 | 0.498124 | 1214738 |
| uniform-0000010-1 | incremental | local_search | 5 | max | 0.1 | - | 0.475684 | 0.533875 | 2 |
| uniform-0000010-2 | incremental | local_search | 5 | min | 0.1 | - | 0.594096 | 0.568609 | 0 |
| uniform-0000050-1 | incremental | local_search | 10 | max | 0.1 | - | 0.521996 | 0.554135 | 92 |
| uniform-0000100-1 | incremental | local_search | 10 | max | 0.1 | - | 0.531515 | 0.535524 | 684 |
| uniform-0000010-1 | incremental | simulated_annealing | 5 | min | - | global | 0.475684 | 0.351727 | 28342 |
| uniform-0000050-1 | incremental | simulated_annealing | 10 | max | - | local | 0.518126 | 0.539427 | 4343 |
| uniform-0000050-2 | incremental | simulated_annealing | 10 | min | - | global | 0.518126 | 0.517346 | 44048 |
| uniform-0000100-1 | incremental | simulated_annealing | 10 | max | - | global | 0.531515 | 0.540023 | 98446 |
| euro-night-0000010 | convex_hull | local_search | 5 | max | 0.1 | - | 0.672225 | 0.863394 | 1 |
| euro-night-0000050 | convex_hull | local_search | 10 | max | 0.1 | - | 0.506611 | 0.51712 | 104 |
| euro-night-0000300 | convex_hull | local_search | 10 | min | 0.1 | - | 0.545025 | 0.538688 | 10645 |
| euro-night-0000010 | convex_hull | simulated_annealing | 5 | max | - | global | 0.672225 | 0.686986 | 491 |
| euro-night-0000050 | convex_hull | simulated_annealing | 10 | min | - | local | 0.506611 | 0.495236 | 1 |
| euro-night-0000300 | convex_hull | simulated_annealing | 100 | max | - | global | 0.545025 | 0.545019 | 281710 |

Παρατηρούμε ότι ο αλγόριθμος `local_search` εκτελείται ταχύτερα από τις παραλλαγές του `simulated_annealing`. Όσον αφορά την βελτιστοποίηση του εμβαδού του πολυγώνου στον `local_search` παρατηρούμε ότι όσο μεγαλώνει ο αριθμός των σημείων η αλλαγή στο εμβαδόν γίνεται μικρότερη. Η επίδοση του `simulated_annealing` εξαρτάται από την αρχική τυχαία επιλογή σημείων και μπορεί να διαφέρει από εκτέλεση σε εκτέλεση. Τέλος παρατηρούμε ότι τα αρχεία `uniform` παρουσιάζουν ελαφρώς χαμηλότερη επίδοση ενώ οι επιδόσεις δεν εξαρτώνται από την αρχική πολυγωνοποίηση σε μεγάλο βαθμό.
