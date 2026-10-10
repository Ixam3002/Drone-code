float borner_valeur(float valeur) {
    if (valeur < 1000.0f) {
        return 1000.0f;
    } 
    if (valeur > 1800.0f) {
        return 1800.0f;
    }
    return valeur;
}